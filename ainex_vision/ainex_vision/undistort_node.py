import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge

import numpy as np
import cv2
import os
import yaml

class UndistortNode(Node):
    def __init__(self):
        super().__init__("undistort_node")
        self.cb_group = ReentrantCallbackGroup()

        self.bridge = CvBridge()
        self.frame = None

        self.raw_frame = None
        self.undistort_frame = None

        # Load YAML
        current_dir = os.path.dirname(__file__)
        yaml_path = os.path.abspath(os.path.join(current_dir, "camera.yaml"))

        self.get_logger().info(f"Loading YAML from: {yaml_path}")

        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        # Camera parameters
        self.K = np.array(data["camera_matrix"]["data"]).reshape(3, 3)
        self.D = np.array(data["distortion_coefficients"]["data"])
        self.height = data["image_height"]
        self.width = data["image_width"]

        # Initialize CameraInfo
        self.cam_info = CameraInfo()
        self.cam_info.width = self.width
        self.cam_info.height = self.height
        self.cam_info.k = self.K.flatten().tolist()
        self.cam_info.d = self.D.flatten().tolist()
        self.cam_info.distortion_model = "plumb_bob"

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber: JPEG compressed input
        self.sub_compressed = self.create_subscription(
            CompressedImage,
            "/camera_image/compressed",
            self.image_callback,
            sensor_qos,
            callback_group=self.cb_group
        )

        # Publishers
        self.pub_img = self.create_publisher(Image, "/camera/image_undistorted", 10)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self.pub_compressed = self.create_publisher(
            CompressedImage, "/camera/image_undistorted/compressed", 10
        )

        # Timer for display loop
        self.create_timer(0.02, self.display_loop)

        self.get_logger().info("UndistortNode started.")

    def image_callback(self, msg: CompressedImage):
        """Receive JPEG → decode → undistort → publish."""
        try:
            # Decode JPEG
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            raw = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if raw is None:
                self.get_logger().warn("JPEG decode failed")
                return
            
            # Save raw for display
            self.raw_frame = raw
            
            # Undistort
            undistorted = cv2.undistort(raw, self.K, self.D)
            self.undistort_frame = undistorted

            # Publish ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(undistorted, "bgr8")
            out_msg.header = msg.header
            self.cam_info.header = msg.header

            self.pub_img.publish(out_msg)
            self.pub_info.publish(self.cam_info)

            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(undistorted)
            compressed_msg.header = msg.header
            self.pub_compressed.publish(compressed_msg)
        
        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")

    def display_loop(self):
        """Show raw vs undistorted comparison."""
        if self.raw_frame is not None and self.undistort_frame is not None:
            # Resize to same height if needed
            h1, w1 = self.raw_frame.shape[:2]
            h2, w2 = self.undistort_frame.shape[:2]

            if h1 != h2:
                self.undistort_frame = cv2.resize(self.undistort_frame, (w1, h1))

            # Side-by-side
            combined = np.hstack((self.raw_frame, self.undistort_frame))

            cv2.imshow("Raw | Undistorted Comparison", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UndistortNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
