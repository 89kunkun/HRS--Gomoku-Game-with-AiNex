# #!/usr/bin/env/ python3
# import rclpy
# from rclpy.node import Node
# import time
# import math

# from ainex_motion.joint_controller import JointController

# class WaveNode(Node):
#     def __init__(self):
#         super().__init__('wave_hand_node')

#         # Use JointController provided by ainex_motion
#         self.joint_controller = JointController(self)

#         # Choose right arm
#         self.shoulder_pitch = 'r_sho_pitch'
#         self.shoulder_roll  = 'r_sho_roll'
#         self.elbow_pitch = 'r_el_pitch'
#         self.elbow_yaw   = 'r_el_yaw'

#         self.home_position()

#         time.sleep(1.0)
#         self.wave()

#     def home_position(self):
#         """Move arm to natural resting position"""
#         self.get_logger().info("Moving to home position...")
#         joints = [self.shoulder_pitch, self.shoulder_roll, self.elbow_pitch, self.elbow_yaw]
#         positions = [0.0, 10.0, 0.0, 90.0]  # rad
#         self.joint_controller.setJointPositions(joints, positions, 1.0, unit='deg')
#         time.sleep(1.0)
    
#     def raise_arm(self):
#         """Raise arm for wave gesture"""
#         self.get_logger().info("Raising arm...")

#         joints = [self.shoulder_pitch, self.shoulder_roll]
#         positions = [90.0, 90.0]  # -40 deg pitch = raise arm forward
#         self.joint_controller.setJointPositions(joints, positions, 1.0, unit='deg')
#         time.sleep(1.5)

#     def wave(self):
#         """Wave gesture with shoulder roll"""
#         self.raise_arm()

#         self.get_logger().info("Waving...")

#         for i in range(4): # wave 4 times
#             # yaw - left
#             self.joint_controller.changeJointPositions([self.elbow_yaw], [50], 0.5, unit='deg')
#             time.sleep(0.5)

#             # yaw - right
#             self.joint_controller.changeJointPositions([self.elbow_yaw], [-50], 0.5, unit='deg')
#             time.sleep(0.5)

#         self.get_logger().info("Wave finished. Returning home...")
#         self.home_position()

# def main(args=None):
#     rclpy.init(args=args)
#     node = WaveNode()
#     # rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import time

class WaveHand:
    def __init__(self, joint_controller):
        self.joint_controller = joint_controller

        self.shoulder_pitch = 'r_sho_pitch'
        self.shoulder_roll  = 'r_sho_roll'
        self.elbow_pitch    = 'r_el_pitch'
        self.elbow_yaw      = 'r_el_yaw'

    def home(self):
        joints = [self.shoulder_pitch, self.shoulder_roll,
                  self.elbow_pitch, self.elbow_yaw]
        positions = [0.0, 10.0, 0.0, 90.0]
        self.joint_controller.setJointPositions(joints, positions, 1.0, unit='deg')
        time.sleep(1.0)

    def raise_arm(self):
        joints = [self.shoulder_pitch, self.shoulder_roll]
        positions = [90.0, 90.0]
        self.joint_controller.setJointPositions(joints, positions, 1.0, unit='deg')
        time.sleep(1.2)

    def wave_once(self):
        self.raise_arm()

        for _ in range(3):
            self.joint_controller.changeJointPositions([self.elbow_yaw], [50], 0.4, unit='deg')
            time.sleep(0.5)
            self.joint_controller.changeJointPositions([self.elbow_yaw], [-50], 0.4, unit='deg')
            time.sleep(0.5)
        self.home()