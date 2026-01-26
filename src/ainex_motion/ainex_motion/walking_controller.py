"""
WalkingController
-----------------
High-level walking interface for AiNex.

Functions:
- activate walking module
- send /cmd_vel commands
- basic walking motions (forward, strafe, rotate, stop)

Used for HRS Exercise 1.
"""

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class WalkingController (Node):
    def __init__(self, node_name: str = "walking_controller"):
        """
        Walking controller node.

        Args:
            node_name (str): ROS2 node name
        """
        super().__init__(node_name)

        # Publisher: cmd_vel
        self.cmd_pub = self.create_publisher(
            Twist, "/cmd_vel", 10
        )
        
        # Service client: activate walking
        self.activate_cli = self.create_client(
            Empty, "/activate_walking"
        )

        self.get_logger().info("WalkingController initailized.")

    # --------------------------------------------------
    # Walking module control
    # --------------------------------------------------
    def activate(self, timeout: float = 5.0):
        """
        Activate walking module and wait until ready.
        """
        self.get_logger().info("Waitiong for /activate_walking service...")
        if not self.activate_cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("activate_walking service not available!")
            return False
        
        req = Empty.Request()
        future = self.activate_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info("Walking module activated.")
        return True
    
    # --------------------------------------------------
    # Low-level cmd_vel sender
    # --------------------------------------------------
    def send_cmd(
            self,
            x: float = 0.0,
            y: float = 0.0,
            yaw : float = 0.0,
            duration : float = 0.0,
            rate : float = 10.0,
    ):
        """
        Publish cmd_vel command.

        Args:
            x (float): linear.x  in [-1, 1]
            y (float): linear.y  in [-1, 1]
            yaw (float): angular.z in [-1, 1]
            duration (float): command duration [s], 0 means send once
            rate (float): publish rate [Hz]
        """
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.angular.z = float(yaw)

        # Single publish
        if duration <= 0.0:
            self.cmd_pub.publish(msg)
            return
        
        period = 1.0 / rate
        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time < duration):
            self.cmd_pub.publish(msg)
            time.sleep(period)
        
    # --------------------------------------------------
    # High-level motions
    # --------------------------------------------------
    def walk_forward(self, v: float = 0.5, duration: float = 2.0):
        """Walk forward (+x)."""
        self.get_logger().info(f"Walking forward: v={v}")
        self.send_cmd(x=v, duration=duration)

    def walk_backward(self, v: float = 0.5, duration: float = 2.0):
        """Walk backward (-x)."""
        self.get_logger().info(f"Walking backward: v={v}")
        self.send_cmd(x=-v, duration=duration)

    def strafe_left(self, v: float = 0.5, duration: float = 2.0):
        """Move left (+y)."""
        self.get_logger().info(f"Strafing left: v={v}")
        self.send_cmd(y=v, duration=duration)

    def strafe_right(self, v: float = 0.5, duration: float = 2.0):
        """Move right (-y)."""
        self.get_logger().info(f"Strafing right: v={v}")
        self.send_cmd(y=-v, duration=duration)

    def rotate_left(self, w: float = 0.5, duration: float = 2.0):
        """Rotate left (+yaw)."""
        self.get_logger().info(f"Rotating left: w={w}")
        self.send_cmd(yaw=w, duration=duration)

    def rotate_right(self, w: float = 0.5, duration: float = 2.0):
        """Rotate right (-yaw)."""
        self.get_logger().info(f"Rotating right: w={w}")
        self.send_cmd(yaw=-w, duration=duration)

    def stop(self):
        """Stop walking."""
        self.get_logger().info("Stopping walking.")
        self.cmd_pub.publish(Twist())
