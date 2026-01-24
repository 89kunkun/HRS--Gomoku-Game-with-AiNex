
from pathlib import Path
from cv2 import aruco
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
from rclpy.callback_groups import ReentrantCallbackGroup

# TEMPLATE_PATH = Path("/home/hrs2025/T3-template/ainex_vision/ainex_vision/T_4Img.png")


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.cb_group = ReentrantCallbackGroup()
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 订阅压缩图像
        self.sub_compressed = self.create_subscription(
            CompressedImage,
            'camera_image/compressed',
            self.image_callback_compressed,
            sensor_qos,
            callback_group=self.cb_group,
        )

        # 订阅 camera_info
        self.sub_camerainfo = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.camera_info_callback,
            sensor_qos,
            callback_group=self.cb_group
        )

        # 状态变量
        self.camera_info_received = False
        self.frame = None
        self.paused = False

        # —— 截图相关设置 ——  （按一次 p 保存一张）
        self.save_dir = Path("/home/hrs2025/Workspace/T2-template/ainex_vision/ainex_vision")
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.save_count = 0          # 已经保存了多少张
        self.save_limit = 20         # 你想采 20 张，可以改
        # ————————————————

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.get_logger().info(
                f'Camera Info received: {msg.width}x{msg.height}\n'
                f'K: {msg.k}\n'
                f'D: {msg.d}'
            )
            print(f'Camera Info received: {msg.width}x{msg.height}')
            print(f'Intrinsic matrix K: {msg.k}')
            print(f'Distortion coeffs D: {msg.d}')
            self.camera_info_received = True

        self.cam_K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.cam_D = np.array(msg.d, dtype=np.float32).reshape(-1)

    # def save_current_as_template(self):
    #     if self.frame is None:
    #         self.get_logger().warn("No camera frame yet.")
    #         return
    #     cv2.imwrite(str(TEMPLATE_PATH), self.frame)
    #     self.get_logger().info(f"Saved current frame -> {TEMPLATE_PATH}")

    def image_callback_compressed(self, msg: CompressedImage):
        try:
            # 解码 JPEG
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn('JPEG decode returned None')
                return

            # 保存当前帧
            self.frame_raw = frame
            self.frame = frame.copy()

        except Exception as exc:
            self.get_logger().error(f'Decode error in compressed image: {exc}')

    def process_key(self):
        key = cv2.waitKey(1) & 0xFF

        # q 退出
        if key == ord('q'):
            return False

        # p 截图一次
        if key == ord('p'):
            if self.frame is None:
                print("[WARN] No frame yet, cannot save!")
            elif self.save_count >= self.save_limit:
                print(f"[INFO] Already saved {self.save_limit} images.")
            else:
                filename = self.save_dir / f"img_{self.save_count:02d}.png"
                ok = cv2.imwrite(str(filename), self.frame)
                if ok:
                    self.save_count += 1
                    print(f"[SAVE] Saved image {self.save_count}/{self.save_limit}: {filename}")
                else:
                    print("[ERROR] Failed to save image!")


        return True

    def display_loop(self):
        while rclpy.ok():
            if (self.frame is not None) and (not self.paused):
                cv2.imshow('Camera Subscriber', self.frame)

            if not self.process_key():
                break

            rclpy.spin_once(self, timeout_sec=0.01)

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    node.get_logger().info('CameraSubscriber node started')

    try:
        node.display_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


