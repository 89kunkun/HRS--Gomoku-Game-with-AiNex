import numpy as np
import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import PoseStamped
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory

from ainex_controller.ainex_model import AiNexModel
from ainex_controller.ainex_robot import AinexRobot
from ainex_controller.ainex_hand_controller import HandController

# ============================================================
# FSM definition
# ============================================================
class ArmState(Enum):
    TRACK = 0
    HOLD = 1
    HOME = 2

# ============================================================
# Initial posture (rad)
# ============================================================
CROUCH_POSTURE = {
    'r_hip_yaw': 0.0, 'r_hip_roll': 0.0, 'r_hip_pitch': 0.70,
    'r_knee': -2.25, 'r_ank_pitch': -1.48, 'r_ank_roll': 0.0,

    'l_hip_yaw': 0.0, 'l_hip_roll': 0.0, 'l_hip_pitch': -0.70,
    'l_knee': 2.25, 'l_ank_pitch': 1.48, 'l_ank_roll': 0.0,

    'r_sho_pitch': 0.6, 'l_sho_pitch': 0.5,
    'l_sho_roll': -0.25, 'r_sho_roll': 0.25,

    'l_el_pitch': 0.08, 'r_el_pitch': -0.08,
    'l_el_yaw': -1.83, 'r_el_yaw': 1.83,

    'l_gripper': 0.0, 'r_gripper': 0.0,
    'head_pan': 0.0, 'head_tilt':  0.1
}

# ============================================================
# MarkerFollower Node
# ============================================================
class MarkerFollower(Node):
    def __init__(self):
        super().__init__('ainex_marker_fsm')

        # --------------------
        # Parameters
        # --------------------
        
        # TRACK -> HOLD
        self.dt = 0.05
        self.spline_duration = 2.0
        self.home_duration = 4.0
        self.reached_tol = 0.01

        self.use_fixed_marker = self.declare_parameter(
            "use_fixed_marker", True
        ).value

        # right hand 
        # self.fixed_marker_pose = np.array(
        #     self.declare_parameter(
        #         "fixed_marker_pose",
        #         ([0.2, -0.1, 0.15])
        #     ).value,
        #     dtype=float
        # )
        
        # left hand
        self.fixed_marker_pose = np.array(
            self.declare_parameter(
                "fixed_marker_pose",
                ([0.2, 0.1, 0.1])
            ).value,
            dtype=float
        )

        # HOLD -> HOME
        self.hold_enter_time = {
            'left': None,
            'right': None
        }
        self.hold_duration = 2.0 # seconds
        self.home_start_time = {'left': None, 'right': None}
        self.max_joint_vel = 0.5
        self.hold_reason = {'left': None, 'right': None}

        # HOME -> TRACK
        self.home_exit_marker = {'left': None, 'right': None}
        self.marker_change_time = {'left': None, 'right': None}

        self.marker_change_thresh = 0.01   # 1 cm
        self.marker_change_hold = 2.0      # 2 seconds

        # sigle arm controll
        self.active_arm = None
        self.swith_hysteresis = 0.01

        # --------------------
        # Load robot
        # --------------------
        pkg = get_package_share_directory('ainex_description')
        urdf_path = pkg + "/urdf/ainex.urdf"
        self.robot_model = AiNexModel(self, urdf_path)

        self.sim_mode = self.declare_parameter("sim", False).value
        self.robot = AinexRobot(self, self.robot_model, self.dt, sim=self.sim_mode)

        # --------------------
        # Initial posture
        # --------------------
        self.q_home = np.zeros(self.robot_model.model.nq)
        for j, v in CROUCH_POSTURE.items():
            self.q_home[self.robot_model.get_joint_id(j)] = v

        self.robot.move_to_initial_position(self.q_home)

        # --------------------
        # Hand controllers
        # --------------------
        self.left_hand = HandController(self, self.robot_model, arm_side='left')
        self.right_hand = HandController(self, self.robot_model, arm_side='right')

        self.arm_home_joints = {
            'left': self.q_home[self.robot_model.get_arm_ids('left')].copy(),
            'right': self.q_home[self.robot_model.get_arm_ids('right')].copy()
        }

        # --------------------
        # FSM states
        # --------------------
        self.arm_state = {
            'left': ArmState.TRACK,
            'right': ArmState.TRACK
        }

        self.marker_pose = None
        self.prev_marker_pose = None

        # --------------------
        # Marker subscription (optional)
        # --------------------
        self.create_subscription(
            PoseStamped,
            "/aruco_marker_base_link",
            self.marker_callback,
            10
        )

        # Control loop
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("FSM MarkerFollower started")

    # ============================================================
    # Marker input
    # ============================================================
    def marker_callback(self, msg: PoseStamped):
        if not self.use_fixed_marker:
            self.marker_pose = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])

    # ============================================================
    # FSM helpers
    # ============================================================
    def reached_target(self, side: str) -> bool:
        ee = (
            self.robot_model.left_hand_pose().translation
            if side == 'left'
            else self.robot_model.right_hand_pose().translation
        )
        return np.linalg.norm(ee - self.marker_pose) < self.reached_tol
    
    def _enter_track_to_marker(self, side: str):
        # reset marker for track
        target = pin.SE3.Identity()
        target.translation = self.marker_pose.copy()
        controller = self.left_hand if side == "left" else self.right_hand
        controller.set_target_pose(target, duration=self.spline_duration, type='abs')

    def _command_joint_home(self, side: str):
        # joint-space controll
        arm_ids = self.robot_model.get_arm_ids(side)
        q_target = self.robot.q.copy()
        q_target[arm_ids] = self.arm_home_joints[side]

        if hasattr(self.robot, "send_cmd"):
            self.robot.send_cmd(q_target, self.home_duration)

        self.robot.q = q_target
        self.robot_model.update_model(self.robot.q, self.robot.v)

    def step_joint_home(self, side: str):
        arm_ids = self.robot_model.get_arm_ids(side)

        q = self.robot.q.copy()
        q_target = q.copy()
        q_home_arm = self.arm_home_joints[side]

        # vel limits
        for i, jid in enumerate(arm_ids):
            dq = q_home_arm[i] - q[jid]
            max_step = self.max_joint_vel * self.dt

            if abs(dq) > max_step:
                dq = np.sign(dq) * max_step

            q_target[jid] = q[jid] + dq

        if hasattr(self.robot, "send_cmd"):
            self.robot.send_cmd(q_target, self.dt)

        self.robot.q = q_target
        self.robot_model.update_model(self.robot.q, self.robot.v)

        if hasattr(self.robot, "publish_joint_states"):
            self.robot.publish_joint_states()

    def _update_active_arm(self):
        if self.marker_pose is None:
            return
    
        y = float(self.marker_pose[1])

        # first select
        if self.active_arm is None:
            self.active_arm = "left" if y > 0 else "right"
            self.get_logger().info(f"Init active_arm = {self.active_arm}")
            return
        
        # hysteresis
        if self.active_arm == "left" and y < -self.swith_hysteresis:
            self._switch_active_arm("right")

        elif self.active_arm == "right" and y > self.swith_hysteresis:
            self._switch_active_arm("left")

    def _switch_active_arm(self, new_arm: str):
        old_arm = self.active_arm
        if new_arm == old_arm:
            return
    
        self.active_arm = new_arm
        self.get_logger().info(f"Switch arm: {old_arm} -> {new_arm}")

        # old arm go back to HOME
        self.arm_state[old_arm] = ArmState.HOME
        self.hold_reason[old_arm] = None
        self.hold_enter_time[old_arm] = None
        self.home_exit_marker[old_arm] = None
        self.marker_change_time[old_arm] = None

        # new arm go to TRACK
        self.arm_state[new_arm] = ArmState.TRACK
        self._enter_track_to_marker(new_arm)

    # ============================================================
    # FSM update (per arm)
    # ============================================================
    def update_arm_fsm(self, side: str):
        controller = self.left_hand if side == "left" else self.right_hand
        state = self.arm_state[side]
        now = self.get_clock().now().nanoseconds * 1e-9

        # =====================================================
        # TRACK
        # =====================================================
        if state == ArmState.TRACK:
            if side != self.active_arm:
                return None
        
            if self.reached_target(side):
                self.arm_state[side] = ArmState.HOLD
                self.hold_enter_time[side] = now
                self.hold_reason[side] = "after_track"
                self.get_logger().info(f"{side} -> HOLD")
                return None
            
            return controller.update(self.dt)
        
        # =====================================================
        # HOLD
        # =====================================================
        elif state == ArmState.HOLD:
            # HOLD after TRACK -> go HOME
            if self.hold_reason[side] == "after_track":
                if (now - self.hold_enter_time[side]) > self.hold_duration:
                    self.arm_state[side] = ArmState.HOME
                    self.hold_enter_time[side] = None
                    self.hold_reason[side] = None

                    self.get_logger().info(f"{side} TRACK -> HOLD (after_track)")
                return controller.update(self.dt)

            # HOLD after HOME → wait marker change
            elif self.hold_reason[side] == "after_home":
                dist = np.linalg.norm(self.marker_pose - self.home_exit_marker[side])

                if dist > self.marker_change_thresh:
                    if self.marker_change_time[side] is None:
                        self.marker_change_time[side] = now

                    elif (now - self.marker_change_time[side]) > self.marker_change_hold:
                        self.arm_state[side] = ArmState.TRACK
                        self.marker_change_time[side] = None
                        self.home_exit_marker[side] = None
                        self.hold_reason[side] = None

                        self._enter_track_to_marker(side)
                        self.get_logger().info(f"{side} MARKER CHANGE -> TRACK")

                else:
                    self.marker_change_time[side] = None

                return None or controller.update(self,dt)
        
        # =====================================================
        # HOME
        # =====================================================
        elif state == ArmState.HOME:
            self.step_joint_home(side)

            arm_ids = self.robot_model.get_arm_ids(side)
            err = np.linalg.norm(
                self.robot.q[arm_ids] - self.arm_home_joints[side]
            )

            if err < 1e-3:
                self.arm_state[side] = ArmState.HOLD
                self.hold_reason[side] = "after_home"
                self.home_exit_marker[side] = self.marker_pose.copy()
                self.marker_change_time[side] = None

                self.get_logger().info(f"{side} HOME done -> HOLD (after_home)")
                return None
            
            return None

        
    # ============================================================
    # Main control loop
    # ============================================================
    def control_loop(self):
        # Marker source
        if self.use_fixed_marker:
            self.marker_pose = self.fixed_marker_pose.copy()

        if self.marker_pose is None:
            return
        
        self._update_active_arm()

        self.robot_model.update_model(self.robot.q, self.robot.v)

        # One-time replan when marker appears
        if self.prev_marker_pose is None:
            self._enter_track_to_marker("left")
            self._enter_track_to_marker("right")
            self.prev_marker_pose = self.marker_pose.copy()
            self.get_logger().info("Initial replan to marker")

        # FSM update
        v_l = None
        v_r = None

        if self.active_arm == "left":
            v_l = self.update_arm_fsm("left")

        elif self.active_arm == "right":
            v_r = self.update_arm_fsm("right")

        if v_l is None and v_r is None:
            return
        
        self.robot.update(v_l, v_r, self.dt)

# ============================================================
# Main
# ============================================================
def main():
    rclpy.init()
    node = MarkerFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()