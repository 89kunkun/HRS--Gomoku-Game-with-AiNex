import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
from ainex_controller.ainex_model import AiNexModel
from ainex_controller.ainex_robot import AinexRobot
from ainex_controller.ainex_hand_controller import HandController

# === Initial posture parameters (rad) ===
CROUCH_POSTURE = {
    'r_hip_yaw':     0.0,
    'r_hip_roll':    0.0,
    'r_hip_pitch':   0.70,
    'r_knee':       -2.25,
    'r_ank_pitch':  -1.48,
    'r_ank_roll':    0.0,

    'l_hip_yaw':    -0.0,
    'l_hip_roll':   -0.0,
    'l_hip_pitch':  -0.70,
    'l_knee':        2.25,
    'l_ank_pitch':   1.48,
    'l_ank_roll':    0.0,

    'r_sho_pitch':   0.6, 
    'l_sho_pitch':   0.5,  # invers
    'l_sho_roll':   -0.25,
    'r_sho_roll':    0.25,

    'l_el_pitch':    0.08,
    'r_el_pitch':   -0.08,
    'l_el_yaw':     -1.83,
    'r_el_yaw':      1.83,

    'l_gripper':     0.0,
    'r_gripper':     0.0,
    'head_pan':      0.0,
    'head_tilt':    -0.8
}

class MarkerFollower(Node):
    def __init__(self):
        super().__init__('ainex_hands_control_node')

        # --------------------
        # Parameters
        # --------------------
        self.dt = 0.05
        self.marker_pose = None
        self.marker_lost_counter = 0
        self.MAX_LOST = 8
        self.prev_marker_pose = None
        self.min_motion_threshold = 0.01
        self.spline_duration = 2.0
        self.replan_period = 0.3
        self.last_replan_time = float("-inf")
        self.last_marker_time = float("-inf")
        self.marker_timeout = self.dt * self.MAX_LOST
        self.home_loss_threshold = 20
        self.dual_band_min = self.declare_parameter('dual_band_min', -0.015).value
        self.dual_band_max = self.declare_parameter('dual_band_max', 0.015).value

        self.right_returning_home = False
        self.left_returning_home = False
        self.active_hand = "right"
        self.home_duration = 4.0

        # --------------------
        # Load robot model
        # --------------------
        pkg = get_package_share_directory('ainex_description')
        urdf_path = pkg + "/urdf/ainex.urdf"
        self.robot_model = AiNexModel(self, urdf_path)

        self.sim_mode = self.declare_parameter("sim", False).value
        self.robot = AinexRobot(self, self.robot_model, self.dt, sim=self.sim_mode)

        # Initial configuration
        self.q_home = np.zeros(self.robot_model.model.nq)
        # self.q_home[self.robot_model.get_joint_id('r_sho_roll')] = 1.4
        # self.q_home[self.robot_model.get_joint_id('l_sho_roll')] = -1.4
        # self.q_home[self.robot_model.get_joint_id('r_el_yaw')] = 1.58
        # self.q_home[self.robot_model.get_joint_id('l_el_yaw')] = -1.58

        # === Build initial configuration from parameter dict ===
        for joint_name, joint_value in CROUCH_POSTURE.items():
            self.q_home[self.robot_model.get_joint_id(joint_name)] = joint_value
    
        self.robot.move_to_initial_position(self.q_home)

        # --------------------
        # Create HandControllers
        # --------------------
        self.right_hand = HandController(self, self.robot_model, arm_side='right')
        self.right_home_pose = self.robot_model.right_hand_pose().copy()

        self.left_hand = HandController(self, self.robot_model, arm_side='left')
        self.left_home_pose = self.robot_model.left_hand_pose().copy()

        self.arm_home_joints = {
            'left': self.q_home[self.robot_model.get_arm_ids('left')],
            'right': self.q_home[self.robot_model.get_arm_ids('right')]
        }

        # --------------------
        # Subscribe to Aruco pose (in base_link) or use fixed marker
        # --------------------
        self.create_subscription(
            PoseStamped,
            "/aruco_marker_base_link",
            self.marker_callback,
            10
        )

        # Control loop timer
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Marker follower ready")

    # --------------------------------------------------------------
    # Receive marker pose
    # --------------------------------------------------------------
    def marker_callback(self, msg: PoseStamped):
        self.marker_lost_counter = 0
        self.last_marker_time = self.get_clock().now().nanoseconds * 1e-9

        # Only translation
        self.marker_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

    # ----------------------------------------------------------------------
    # Main control loop
    # ----------------------------------------------------------------------
    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        self.robot_model.update_model(self.robot.q, self.robot.v)

        if self.marker_pose is not None and (now - self.last_marker_time) > self.marker_timeout:
            self.marker_pose = None
            self.prev_marker_pose = None

        marker_available = self.marker_pose is not None

        if marker_available:
            y = self.marker_pose[1]
            if y > self.dual_band_max:
                desired_mode = "left"
            elif y < self.dual_band_min:
                desired_mode = "right"
            else:
                desired_mode = "both"

            if desired_mode != self.active_hand:
                self.active_hand = desired_mode
                self.prev_marker_pose = None
                self.last_replan_time = now - self.replan_period - 1.0

            if self.active_hand == "both":
                active_sides = ["left", "right"]
            elif self.active_hand == "left":
                active_sides = ["left"]
            else:
                active_sides = ["right"]

            for side in active_sides:
                self._cancel_home_motion(side)
            for side in {"left", "right"} - set(active_sides):
                self._start_home_motion(side)

            movement = np.inf
            if self.prev_marker_pose is not None:
                movement = np.linalg.norm(self.marker_pose - self.prev_marker_pose)

            time_since_plan = now - self.last_replan_time
            should_replan = (
                movement > self.min_motion_threshold or
                time_since_plan > self.replan_period
            )

            if should_replan:
                target = pin.SE3.Identity()
                target.translation = self.marker_pose.copy()
                for side in active_sides:
                    controller = self.left_hand if side == "left" else self.right_hand
                    controller.set_target_pose(target, duration=self.spline_duration, type='abs')
                self.prev_marker_pose = self.marker_pose.copy()
                self.last_replan_time = now
                self.get_logger().info(f"Replan! hand={self.active_hand} marker={self.marker_pose}")
        else:
            self.marker_lost_counter += 1
            if self.marker_lost_counter >= self.home_loss_threshold:
                self._start_home_motion("left")
                self._start_home_motion("right")

        v_cmd_left = None
        v_cmd_right = None

        left_active = marker_available and self.active_hand in ("left", "both")
        right_active = marker_available and self.active_hand in ("right", "both")

        if self.left_returning_home:
            if self.left_hand.motion_complete():
                self.left_returning_home = False
                self.left_hand.ignore_manipulability = False
            else:
                v_cmd_left = self.left_hand.update(self.dt)
        elif left_active:
            v_cmd_left = self.left_hand.update(self.dt)

        if self.right_returning_home:
            if self.right_hand.motion_complete():
                self.right_returning_home = False
                self.right_hand.ignore_manipulability = False
            else:
                v_cmd_right = self.right_hand.update(self.dt)
        elif right_active:
            v_cmd_right = self.right_hand.update(self.dt)

        if v_cmd_left is None and v_cmd_right is None:
            return

        self.robot.update(v_cmd_left, v_cmd_right, self.dt)

    def _hand_objects(self, side: str):
        if side == "left":
            return self.left_hand, self.left_home_pose, "left_returning_home"
        return self.right_hand, self.right_home_pose, "right_returning_home"

    def _start_home_motion(self, side: str):
        controller, home_pose, flag_attr = self._hand_objects(side)
        if getattr(self, flag_attr):
            return
        if self._arm_is_at_home(side):
            return
        if self._command_joint_home(side):
            return
        controller.set_target_pose(home_pose, duration=self.home_duration, type='abs')
        controller.ignore_manipulability = True
        setattr(self, flag_attr, True)

    def _cancel_home_motion(self, side: str):
        controller, _, flag_attr = self._hand_objects(side)
        if getattr(self, flag_attr):
            setattr(self, flag_attr, False)
            controller.ignore_manipulability = False

    def _arm_is_at_home(self, side: str, tol: float = 1e-3) -> bool:
        ids = self.robot_model.get_arm_ids(side)
        return np.linalg.norm(self.robot.q[ids] - self.arm_home_joints[side]) <= tol

    def _command_joint_home(self, side: str) -> bool:
        if self.sim_mode or not hasattr(self.robot, 'joint_controller'):
            return False
        q_target = self.robot.q.copy()
        arm_ids = self.robot_model.get_arm_ids(side)
        q_target[arm_ids] = self.arm_home_joints[side]
        self.robot.send_cmd(q_target, self.home_duration)
        self.robot.q = q_target
        self.robot_model.update_model(self.robot.q, self.robot.v)
        self.robot.publish_joint_states()
        return True


def main():
    rclpy.init()
    node = MarkerFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
