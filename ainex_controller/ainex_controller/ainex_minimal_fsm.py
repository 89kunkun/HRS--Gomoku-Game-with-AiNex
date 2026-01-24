#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from enum import Enum
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory

from ainex_controller.ainex_model import AiNexModel
from ainex_controller.ainex_robot import AinexRobot
from ainex_controller.ainex_hand_controller import HandController


# ============================================================
# FSM
# ============================================================
class ArmState(Enum):
    MOVE = 0
    STOP = 1


# ============================================================
# Initial posture
# ============================================================
CROUCH_POSTURE = {
    'r_hip_yaw': 0.0, 'r_hip_roll': 0.0, 'r_hip_pitch': 0.7,
    'r_knee': -2.25, 'r_ank_pitch': -1.48, 'r_ank_roll': 0.0,
    'l_hip_yaw': 0.0, 'l_hip_roll': 0.0, 'l_hip_pitch': -0.7,
    'l_knee': 2.25, 'l_ank_pitch': 1.48, 'l_ank_roll': 0.0,
    'r_sho_pitch': 0.6, 'l_sho_pitch': 0.5,
    'l_sho_roll': -0.25, 'r_sho_roll': 0.25,
    'l_el_pitch': 0.08, 'r_el_pitch': -0.08,
    'l_el_yaw': -1.83, 'r_el_yaw': 1.83,
    'l_gripper': 0.0, 'r_gripper': 0.0,
    'head_pan': 0.0, 'head_tilt': -0.8
}


# ============================================================
# Node
# ============================================================
class MinimalFSM(Node):
    def __init__(self):
        super().__init__('ainex_minimal_fsm')

        self.dt = 0.05

        # 固定 marker（世界坐标 / base_link）
        self.marker_pose = np.array([0.2, -0.1, -0.13])

        # --------------------
        # Robot
        # --------------------
        pkg = get_package_share_directory('ainex_description')
        urdf_path = pkg + "/urdf/ainex.urdf"
        self.robot_model = AiNexModel(self, urdf_path)

        self.robot = AinexRobot(
            self,
            self.robot_model,
            self.dt,
            sim=self.declare_parameter("sim", False).value
        )

        # --------------------
        # Initial posture
        # --------------------
        q0 = np.zeros(self.robot_model.model.nq)
        for j, v in CROUCH_POSTURE.items():
            q0[self.robot_model.get_joint_id(j)] = v
        self.robot.move_to_initial_position(q0)

        # --------------------
        # Hand controller（只用右手，降低复杂度）
        # --------------------
        self.hand = HandController(self, self.robot_model, arm_side='right')

        # FSM
        self.state = ArmState.MOVE
        self.has_planned = False

        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("=== Minimal FSM started ===")

    # ============================================================
    # Control loop
    # ============================================================
    def control_loop(self):
        self.robot_model.update_model(self.robot.q, self.robot.v)

        # --------------------
        # One-time planning
        # --------------------
        if not self.has_planned:
            target = pin.SE3.Identity()
            target.translation = self.marker_pose.copy()

            self.hand.set_target_pose(
                target,
                duration=3.0,
                type='abs'
            )

            self.has_planned = True
            self.get_logger().warn(
                f"Planning to fixed marker {self.marker_pose}"
            )

        # --------------------
        # FSM
        # --------------------
        if self.state == ArmState.MOVE:
            v = self.hand.update(self.dt)

            if v is None:
                self.get_logger().warn("Hand returned None velocity")
                return

            # 关键：一定发命令
            self.robot.update(
                None,     # left arm unused
                v,        # right arm velocity
                self.dt
            )


# ============================================================
# Main
# ============================================================
def main():
    rclpy.init()
    node = MinimalFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
