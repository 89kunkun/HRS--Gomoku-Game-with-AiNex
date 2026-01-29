import numpy as np
import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import PointStamped
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

    'l_gripper': 1.0, 'r_gripper': -1.0,
    'head_pan': 0.0, 'head_tilt':  -0.1
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
        self.dt = 0.05
        self.spline_duration = 2.0
        self.home_duration = 4.0
        self.reached_tol = 0.002
        self.max_joint_vel = self.declare_parameter(
            "max_joint_vel", 0.8
        ).value
        self.hold_duration = self.declare_parameter(
            "hold_duration", 2.0
        ).value
        self.marker_change_thresh = self.declare_parameter(
            "marker_change_thresh", 0.01
        ).value
        self.marker_change_hold = self.declare_parameter(
            "marker_change_hold", 0.5
        ).value
        self.pre_to_final_wait = self.declare_parameter(
            "pre_to_final_wait", 0.5
        ).value

        self.offset_x = self.declare_parameter("offset_x", -0.28).value
        self.offset_y = self.declare_parameter("offset_y", 0.15).value
        self.offset_z = self.declare_parameter("offset_z", 0.06).value

        # pre_point offset along the x
        self.pre_dx = self.declare_parameter("pre_dx", 0.03).value  # meter
        self.pre_reached_tol = self.declare_parameter("pre_reached_tol", 0.0001).value 

        self.use_fixed_marker = self.declare_parameter(
            "use_fixed_marker", False
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
                ([0.20, 0.1, 0.15])
            ).value,
            dtype=float
        )

        # runtime marker
        self.current_marker_pose = self.fixed_marker_pose.copy()
        self.marker_pose = None
        self.latest_topic_pose = None

        # FSM bookkeeping
        self.active_arm = None
        self.swith_hysteresis = 0.01
        self.need_reselect_arm = False

        self.hold_enter_time = {'left': None, 'right': None}
        self.hold_reason = {'left': None, 'right': None}
        self.home_exit_marker = {'left': None, 'right': None}
        self.marker_change_time = {'left': None, 'right': None}
        self.home_exit_marker_seq = {'left': 0, 'right': 0}
        self.marker_seq = 0
        self.marker_update_start_time = None

        # pre-arm track phse (0: go pre, 1: go final)
        self.track_phase = {'left': 0, 'right': 0}
        self.track_done = {'left': False, 'right': False}
        self.pre_waiting = {'left': False, 'right': False}
        self.pre_wait_start = {'left': None, 'right': None}

        # --------------------
        # Load robot
        # --------------------
        pkg = get_package_share_directory('ainex_description')
        urdf_path = pkg + "/urdf/ainex.urdf"

        self.robot_model = AiNexModel(self, urdf_path)
        # =======================================================
        self.sim_mode = self.declare_parameter("sim", False).value
        # =======================================================
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
        self.prev_marker_pose = None

        # --------------------
        # Marker subscription (optional)
        # --------------------
        self.create_subscription(
            PointStamped,
            "/piece_point_base_from_ij",
            self.marker_callback,
            10
        )

        # Control loop
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("FSM MarkerFollower started")

    # ============================================================
    # Marker input
    # ============================================================
    def marker_callback(self, msg: PointStamped):
        if not self.use_fixed_marker:
            pose = np.array([
                msg.point.x + float(self.offset_x),
                msg.point.y + float(self.offset_y),
                msg.point.z + float(self.offset_z)
            ])
            if self.marker_pose is None:
                # initialize once to start FSM
                self.marker_pose = pose.copy()
            else:
                # only update after HOME
                self.latest_topic_pose = pose.copy()

    def _sample_new_marker(self):
        """Sample a new marker pose after HOME."""
        x = 0.2
        # y ∈ [-0.10, 0.10] step 0.01
        y_idx = np.random.randint(-10, 11)   # [-10, 10]
        y = 0.01 * y_idx
        # z ∈ [0.01, 0.18] step 0.01
        z_idx = np.random.randint(1, 19)     # [1, 18]
        z = 0.01 * z_idx

        self.current_marker_pose = np.array([x, y, z], dtype=float)
        self.marker_pose = self.current_marker_pose.copy()
        self.get_logger().info(
            f"New marker sampled: x={x:.2f}, y={y:.3f}, z={z:.3f}"
        )

    # ============================================================
    # Active arm selection
    # ============================================================

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
        self.arm_state[old_arm] = ArmState.HOME
        self.hold_reason[old_arm] = None
        self.hold_enter_time[old_arm] = None
        self.pre_waiting[old_arm] = False
        self.pre_wait_start[old_arm] = None
        
        # reset phase for new active arm
        self.track_phase[new_arm] = 0
        self.pre_waiting[new_arm] = False
        self.pre_wait_start[new_arm] = None

        # new arm go to TRACK
        self.arm_state[new_arm] = ArmState.TRACK
        self._enter_track_to_marker(new_arm)
        self.get_logger().info(f"Switch arm: {old_arm} -> {new_arm}")

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
    
    def _compute_pre_marker(self) -> np.ndarray:
        m = self.marker_pose.copy()
        m[0] = m[0] - float(self.pre_dx)    # move backward along +x axis direction
        return m
    
    # MODIFIED: enter track uses phase to choose traget
    def _enter_track_to_marker(self, side: str):
        if self.marker_pose is None:
            return
        
        if self.track_phase[side] == 0:
            tgt_xyz = self._compute_pre_marker()
        else:
            tgt_xyz = self.marker_pose.copy()
        
        # reset marker for track
        target = pin.SE3.Identity()
        target.translation = tgt_xyz
        controller = self.left_hand if side == "left" else self.right_hand
        controller.set_target_pose(target, duration=self.spline_duration, type='abs')

    def _stop_controller(self, side: str):
        controller = self.left_hand if side == "left" else self.right_hand
        controller.spline = None
        controller.start_time = None

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

        if not self.robot.sim:
            self.robot.send_cmd(q_target, self.dt)

        self.robot.q = q_target
        self.robot_model.update_model(self.robot.q, self.robot.v)  

    def _command_joint_home(self, side: str):
        # joint-space controll
        arm_ids = self.robot_model.get_arm_ids(side)
        q_target = self.robot.q.copy()
        q_target[arm_ids] = self.arm_home_joints[side]

        if not self.robot.sim:
            self.robot.send_cmd(q_target, self.home_duration)

        self.robot.q = q_target
        self.robot_model.update_model(self.robot.q, self.robot.v)

    def _enter_home(self, side: str):
        self._stop_controller(side)
        self.arm_state[side] = ArmState.HOME
        self.pre_waiting[side] = False
        self.pre_wait_start[side] = None

    def _singularity_abort(self, side: str) -> bool:
        Controller = self.left_hand if side == "left" else self.right_hand
        return getattr(Controller, "in_singularity", False)

    # ============================================================
    # FSM update (per arm)
    # ============================================================
    def update_arm_fsm(self, side: str):
        controller = self.left_hand if side == "left" else self.right_hand
        state = self.arm_state[side]
        now = self.get_clock().now().nanoseconds * 1e-9

        # =====================================================
        # Singularity -> FORCE HOME immediately
        # =====================================================
        if self._singularity_abort(side) and state != ArmState.HOME:
            self.get_logger().warn(f"{side} singularity -> FORCE HOME")
            self._enter_home(side)
            self.track_phase[side] = 0
            controller.in_singularity = False
            self.hold_reason[side] = None
            self.hold_enter_time[side] = None
            return None

        # =====================================================
        # TRACK
        # =====================================================
        if state == ArmState.TRACK:
            if self.pre_waiting[side]:
                if (now - self.pre_wait_start[side]) >= self.pre_to_final_wait:
                    self.pre_waiting[side] = False
                    self.pre_wait_start[side] = None
                    self._enter_track_to_marker(side)
                    self.get_logger().info(
                        f"{side} PRE wait done -> continue to FINAL"
                    )
                return None

            if side != self.active_arm:
                # Avoid non-active arm staying in TRACK forever.
                self._stop_controller(side)
                self.arm_state[side] = ArmState.HOLD
                self.hold_reason[side] = "after_track"
                self.hold_enter_time[side] = now
                return None
            
            # MODIFIED: two-stage TRACK logic
            tol = self.pre_reached_tol if self.track_phase[side] == 0 else self.reached_tol
            if controller.motion_complete(tol):
                self._stop_controller(side)

                if self.track_phase[side] == 0:
                    # phase0 finished -> go phase1 (final marker)
                    self.track_phase[side] = 1
                    self.pre_waiting[side] = True
                    self.pre_wait_start[side] = now
                    self.get_logger().info(
                        f"{side} reached PRE -> wait {self.pre_to_final_wait:.2f}s"
                    )
                    return None
                else:
                    # phase1 finished -> mark done
                    self.track_done[side] = True
                    return None
            
            if self.track_done[side]:
                self.track_done[side] = False
                self.track_phase[side] = 0
                self.arm_state[side] = ArmState.HOLD
                self.hold_reason[side] = "after_track"
                self.hold_enter_time[side] = now
                self.get_logger().info(f"{side} TRACK(final) -> HOLD")
                return None
            
            return controller.update(self.dt)
        
        # =====================================================
        # HOLD
        # =====================================================
        elif state == ArmState.HOLD:

            # HOLD after TRACK -> go HOME
            if self.hold_reason[side] == "after_track":
                if (now - self.hold_enter_time[side]) > self.hold_duration:
                    self.hold_reason[side] = None
                    self.hold_enter_time[side] = None
                    self._enter_home(side)
                    self.get_logger().info(f"{side} HOLD(after_track)-> HOME")
                return None

            # HOLD after HOME → wait marker update
            elif self.hold_reason[side] == "after_home":
                if self.marker_seq > self.home_exit_marker_seq[side]:
                    if side == self.active_arm:
                        self.marker_change_time[side] = None
                        self.home_exit_marker[side] = None
                        self.hold_reason[side] = None
                        self.arm_state[side] = ArmState.TRACK

                        # start from phase0 again
                        self.track_phase[side] = 0
                        self._enter_track_to_marker(side)
                        self.get_logger().info(f"{side} HOLD(after_home) -> TRACK")
                    else:
                        # Not active: wait for the next marker update.
                        self.home_exit_marker_seq[side] = self.marker_seq
                return None
            
            return None
        
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
                if side == self.active_arm:
                    # sample a new marker only when active arm finishes HOME
                    if self.use_fixed_marker:
                        self._sample_new_marker()
                    else:
                        if self.latest_topic_pose is not None:
                            self.marker_pose = self.latest_topic_pose.copy()
                            self.latest_topic_pose = None
                            self.marker_seq += 1
                            self.prev_marker_pose = self.marker_pose.copy()
                    self.active_arm = None
                    self._update_active_arm()

                self.arm_state[side] = ArmState.HOLD
                self.hold_reason[side] = "after_home"
                self.home_exit_marker[side] = (
                    self.marker_pose.copy() if self.marker_pose is not None else None
                )
                self.home_exit_marker_seq[side] = self.marker_seq
                self.marker_change_time[side] = None

                # reset phase when ariving home
                self.track_phase[side] = 0

                self.get_logger().info(
                    f"{side} HOME done -> HOLD(after_home), "
                    f"new active_arm = {self.active_arm}"
                )
            return None

    # ============================================================
    # Main control loop
    # ============================================================
    def control_loop(self):
        # Marker source
        if self.use_fixed_marker:
            self.marker_pose = self.current_marker_pose.copy()
        if self.marker_pose is None:
            return
        
        self._update_active_arm()
        self.robot_model.update_model(self.robot.q, self.robot.v)

        # Track marker changes
        if self.prev_marker_pose is None:
            self.prev_marker_pose = self.marker_pose.copy()
            # Initial replan (both arm)
            self.track_phase['left'] = 0
            self.track_phase['right'] = 0
            # One-time replan when marker appears
            self._enter_track_to_marker("left")
            self._enter_track_to_marker("right")
            self.get_logger().info("Initial replan to marker")
        else:
            dist = np.linalg.norm(self.marker_pose - self.prev_marker_pose)
            if dist > self.marker_change_thresh:
                if self.marker_update_start_time is None:
                    self.marker_update_start_time = self.get_clock().now().nanoseconds * 1e-9
                elif (self.get_clock().now().nanoseconds * 1e-9 - self.marker_update_start_time) > self.marker_change_hold:
                    self.marker_seq += 1
                    self.prev_marker_pose = self.marker_pose.copy()
                    self.marker_update_start_time = None
            else:
                self.marker_update_start_time = None

        # FSM update
        v_l = None
        v_r = None

        v_l = self.update_arm_fsm("left")
        v_r = self.update_arm_fsm("right")

        if v_l is None and v_r is None:
            return
        if v_l is None:
            v_l = np.zeros(len(self.robot_model.get_arm_ids("left")))
        if v_r is None:
            v_r = np.zeros(len(self.robot_model.get_arm_ids("right")))

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
