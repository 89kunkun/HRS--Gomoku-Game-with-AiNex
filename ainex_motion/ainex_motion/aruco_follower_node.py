"""
aruco_follower_node
-------------------
Follow an ArUco marker using walking controller.

Subscribes:
- /aruco_marker_base_link (PoseStamped)

Controls:
- /cmd_vel via WalkingController

States:
- SEARCH : head scanning, base stopped
- FOLLOW : base walking, head fixed
- STOP   : base stopped, head fixed
"""

import time
import math
import threading
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from ainex_motion.walking_controller import WalkingController
from ainex_motion.joint_controller import JointController

# ============================================================
# Head scanning behavior (SEARCH)
# ============================================================
class HeadShake:
    def __init__(self, joint_controller):
        self.joint_controller = joint_controller
        self.stop_flag = False
        self.thread = None

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return
        self.stop_flag = False
        self.thread = threading.Thread(
            target=self._shake_loop, daemon=True
        )
        self.thread.start()

    def stop(self):
        self.stop_flag = True

    def _shake_loop(self):
        joint_names = ['head_pan', 'head_tilt']

        # initial position
        self.joint_controller.setJointPositions(
            joint_names, [0.0, 0.0], 1.0, unit='deg'
        )
        time.sleep(0.8)

        t0 = time.time()
        while not self.stop_flag:
            t = time.time() - t0
            pan = 1.0 * math.sin(0.5 * t)  # sin screen

            self.joint_controller.setJointPositions(
                joint_names,
                [pan, 0.0],
                0.02,
                unit='rad'
            )
            time.sleep(0.02)

class State(Enum):
    SEARCH = 0
    FOLLOW = 1
    STOP = 2

class ArucoFollower(Node):
    def __init__(self):
        super().__init__("aruco_follower")

        # Walking controller (backend)
        self.walker = WalkingController(node_name="walking_controller_from_aruco")
        self.walker.activate()

        self.joint_controller = JointController(self)
        self.head_shake = HeadShake(self.joint_controller)
        self.head_shaking = False

        # FSM state
        self.state = State.SEARCH

        # Marker data
        self.last_marker_time = None
        self.marker_pose = None
        self.marker_timeout = 0.5    # [s] marker lost threshold

        # SEARCH behavior
        self.search_yaw = 0.3        # rad/s

        # Subscribe marker pose in base_link
        self.create_subscription(
            PoseStamped,
            "/aruco_marker_base_link",
            self.marker_callback,
            10
        )

        # Control parameters
        self.k_x = 0.8         # forward gain
        self.k_yaw = 1.5       # yaw gain
        self.x_target = 0.1         # desired distance [m]
        self.max_v = 0.6
        self.max_w = 0.8

        # FSM update timer
        self.create_timer(0.05, self.fsm_step)

        self.get_logger().info("Aruco follower started.")

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------
    def marker_callback(self, msg: PoseStamped):
        self.marker_pose = msg
        self.last_marker_time = time.time()

    # --------------------------------------------------
    # FSM logic
    # --------------------------------------------------
    def fsm_step(self):
        now = time.time()

        marker_visible = (
            self.last_marker_time is not None
            and (now - self.last_marker_time) < self.marker_timeout
        )

        # ---------------- SEARCH ----------------
        if self.state == State.SEARCH:
            if marker_visible:
                self.get_logger().info("Marker found → FOLLOW")

                if self.head_shaking:
                    self.head_shake.stop()
                    self.head_shaking = False

                self.state = State.FOLLOW
                return

            # head shake
            if not self.head_shaking:
                self.head_shake.start()
                self.head_shaking = True
                self.get_logger().info("SEARCH: head scanning")

            # base not move
            self.walker.stop()
            return

        # ---------------- FOLLOW ----------------
        if self.state == State.FOLLOW:
            if not marker_visible:
                self.get_logger().warn("Marker lost → SEARCH")
                self.state = State.SEARCH
                return

            x = self.marker_pose.pose.position.x
            y = self.marker_pose.pose.position.y

            ex = x - self.x_target
            ey = y

            # Reached target
            if abs(ex) < 0.05 and abs(ey) < 0.05:
                self.get_logger().info("Target reached → STOP")
                self.walker.stop()
                self.state = State.STOP
                return

            # Control law
            v = self.k_x * ex
            w = self.k_yaw * ey

            # Saturation
            v = max(min(v, self.max_v), -self.max_v)
            w = max(min(w, self.max_w), -self.max_w)

            self.walker.send_cmd(x=v, yaw=w)
            return

        # ---------------- STOP ----------------
        if self.state == State.STOP:
            if self.head_shaking:
                self.head_shake.stop()
                self.head_shaking = False

            self.walker.stop()

            # Optional: if marker moves away, follow again
            if marker_visible:
                x = self.marker_pose.pose.position.x
                y = self.marker_pose.pose.position.y
                if abs(x - self.x_target) > 0.1 or abs(y) > 0.1:
                    self.get_logger().info("Marker moved → FOLLOW")
                    self.state = State.FOLLOW
            return

def main():
    rclpy.init()
    node = ArucoFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
