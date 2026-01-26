# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import time
# import math

# from ainex_motion.joint_controller import JointController

# class HeadShakeNode(Node):
#     def __init__(self):
#         super().__init__('shake_head_node')

#         # Initialize motion controller
#         self.joint_controller = JointController(self)

#         # Head joints
#         self.head_pan = 'head_pan'    # left-right
#         self.head_tilt = 'head_tilt'  # up-down
#         self.stop_flag = False
#         # Move to home first
#         self.home_position()
#         time.sleep(1.0)

#         # Start shaking
#         # self.shake_head()
#         self.can_stop_shake()
#         # Exit node 
#         self.get_logger().info("Head shake finished. Exiting...")
#         rclpy.shutdown()
#     def home_position(self):
#         """Set head to neutral position."""
#         self.get_logger().info("Moving head to home position...")
#         joints = [self.head_pan, self.head_tilt]
#         positions = [0.0, 0.0] # center (rad)
#         self.joint_controller.setJointPositions(joints, positions, duration=1.0, unit='deg')
#         time.sleep(1.0)

#     def shake_head(self):
#         """Left-right head shake."""
#         self.get_logger().info("Shaking head (left-right)...")

#         amplitude = 30  # +-25 degree swing
#         duration  = 1.5 # time for each half-swing

#         for i in range(4):  # shake 4 times
#             # look left
#             self.joint_controller.setJointPositions(
#                 [self.head_pan],
#                 [amplitude],
#                 duration=duration,
#                 unit='deg'
#             )
#             time.sleep(duration)

#             # look right
#             self.joint_controller.setJointPositions(
#                 [self.head_pan],
#                 [-amplitude],
#                 duration=duration,
#                 unit='deg'
#             )
#             time.sleep(duration)
        
#         # Return to home
#         self.home_position()
    
#     def can_stop_shake(self):
#         joint_names = ['head_pan', 'head_tilt']
#     # Set initial joint positions to 0 degrees (converted to radians in setJointPositions)
#         self.joint_controller.setJointPositions(joint_names, [0] * len(joint_names), 1, unit='deg')
#         time.sleep(1.5)
#         t0 = time.time()
#         try:
#             while True:
#                 # Update joint position command based on a sine wave function
#                 t = time.time() - t0
#                 set_position = 1 * math.sin((1) * t)
#                 if self.stop_flag : break
#                 # self.joint_controller.setJointPositions(joint_names, [set_position]* len(joint_names), 0.01, unit='rad')
#                 self.joint_controller.setJointPositions(joint_names, [set_position,0], 0.01, unit='rad')
#                 ### Note: Reading is SLOW. (better avoid reading in a loop)
#                 # position = joint_controller.getJointPositions(joint_names, unit='deg')
#                 # node.get_logger().info(f"Received Position: {position}")  
#         except KeyboardInterrupt:
#             self.joint_controller.setJointPositions(joint_names, [0] * len(joint_names), 1, unit='deg')
#             time.sleep(1.5)
#             self.joint_controller.setJointLock(joint_names, False)
#             time.sleep(1.5)



# def main(args=None):
#     rclpy.init(args=args)
#     node = HeadShakeNode()

# if __name__ == '__main__':
#     main()

# head_shake.py
import time
import math
import threading

class HeadShake:
    def __init__(self, joint_controller):
        self.joint_controller = joint_controller
        self.stop_flag = False
        self.thread = None

    def start(self):
        self.stop_flag = False
        self.thread = threading.Thread(target=self._shake_loop)
        self.thread.start()

    def stop(self):
        self.stop_flag = True

    def _shake_loop(self):
        joint_names = ['head_pan', 'head_tilt']
        self.joint_controller.setJointPositions(joint_names, [0, 0], 1, unit='deg')
        time.sleep(0.8)

        t0 = time.time()
        while not self.stop_flag:
            t = time.time() - t0
            set_position = 1.0 * math.sin(1.0 * t)

            self.joint_controller.setJointPositions(
                joint_names,
                [set_position, 0],
                0.02,
                unit='rad'
            )
            time.sleep(0.02)
