
import rclpy
from rclpy.node import Node
import threading
from servo_service.msg import FaceBoundingBoxArray

from ainex_motion.joint_controller import JointController
from ainex_social_robot.shake_head_node import HeadShake
from ainex_social_robot.wave_hand_node import WaveHand


class SocialNode(Node):
    def __init__(self):
        super().__init__("social_node")

        # robot joints controller
        self.joint_controller = JointController(self)

        # new non-node classes
        self.head_shake = HeadShake(self.joint_controller)
        self.wave_hand = WaveHand(self.joint_controller)

        # status flags
        self.busy = False
        self.shaking = False
        self.face_detect_start = None
        self.waiting_confirm = False
        self.face_confirm_sec = 1.0

        # start head shake
        self._start_head_shake()

        # subscribe face detection
        self.create_subscription(
            FaceBoundingBoxArray,
            "/mediapipe/face_bbox",
            self.face_callback,
            10
        )

        self.get_logger().info("SocialNode started.")


    # -------------------------------
    #     HEAD SHAKE CONTROL
    # -------------------------------
    def _start_head_shake(self):
        if not self.shaking:
            self.shaking = True
            self.head_shake.stop_flag = False
            threading.Thread(target=self.head_shake.start).start()


    def _stop_head_shake(self):
        self.shaking = False
        self.head_shake.stop()


    # -------------------------------
    #      FACE CALLBACK
    # -------------------------------
    def face_callback(self, msg):
        # reset/continue head shake when no faces
        if len(msg.boxes) == 0:
            self.face_detect_start = None
            if self.waiting_confirm and not self.busy:
                self.waiting_confirm = False
                self.get_logger().info("Face lost before 1s, resume head shake.")
                self._start_head_shake()
            return

        # already waving
        if self.busy:
            return

        now = self.get_clock().now()
        if not self.waiting_confirm:
            self.waiting_confirm = True
            self.face_detect_start = now
            self.get_logger().info("Face detected, stopping head shake immediately.")
            self._stop_head_shake()
            return

        elapsed = (now - self.face_detect_start).nanoseconds / 1e9
        if elapsed < self.face_confirm_sec:
            return

        self.busy = True
        self.waiting_confirm = False
        self.face_detect_start = None
        self.get_logger().info("Face stable for 1s, start waving.")

        # do wave asynchronously
        threading.Thread(target=self._do_wave_and_restart).start()


    # -------------------------------
    #     WAVE + RESTART SHAKE
    # -------------------------------
    def _do_wave_and_restart(self):
        # perform wave
        self.wave_hand.wave_once()

        # restart shaking after done
        self.get_logger().info("Wave done, restarting head shake...")
        self._start_head_shake()

        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = SocialNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
        

            
            


        
