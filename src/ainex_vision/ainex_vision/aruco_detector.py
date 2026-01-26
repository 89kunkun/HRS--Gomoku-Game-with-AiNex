import numpy as np
import cv2
from cv2 import aruco
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped

from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.br = TransformBroadcaster(self)
        self.bridge = CvBridge()

        # TF" Buffer + Listener for transforming to base_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe camera image and camera info
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(CompressedImage, '/camera/image_undistorted/compressed', self.image_callback, image_qos)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, image_qos)

        # Publisher for Aruco pose
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        self.marker_pub = self.create_publisher(PoseStamped, "/aruco_marker_base_link", 10)

        # Camera info storage
        self.K = None
        self.D = None

        # Aruco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.marker_length = 0.049  # 5cm marker

    def camera_info_callback(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)
        self.get_logger().info("Camera intrinsics received.")

    def image_callback(self, msg: CompressedImage):
        if self.K is None:
            return
        
        # Decode compressed image
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect Aruco markers
        corners, ids, _= aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if ids is None:
            return
        
        # Estimate pose
        rvec, tvec, _= aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.K, self.D
        )
        t = tvec[0][0]
        r = rvec[0][0]

        # Convert rotation
        R, _ = cv2.Rodrigues(r)
        quat = self.rotation_matrix_to_quaternion(R)

        # ---------------------- Publish pose in camera frame ----------------------
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "camera_optical_link"
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.position.x = float(t[0])
        pose_msg.pose.position.y = float(t[1])
        pose_msg.pose.position.z = float(t[2])

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pose_pub.publish(pose_msg)

        # ---------------------- Publish TF ----------------------
        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = 'aruco_marker'

        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])

        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        # tf_msg.transform.translation = pose_msg.pose.position
        # tf_msg.transform.rotation = pose_msg.pose.orientation

        self.br.sendTransform(tf_msg)

        # ---------------------- Transform to base_link ----------------------
        try:
            # 1) lookup the transform: base_link <- camera_optical_link
            T_base_camera = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_optical_link",
                rclpy.time.Time()
            )

            # quaternion base<-camera
            quat_bc = [
                T_base_camera.transform.rotation.x,
                T_base_camera.transform.rotation.y,
                T_base_camera.transform.rotation.z,
                T_base_camera.transform.rotation.w
            ]

            # Convert TF into matrix
            # rotation matrix
            R_bc = self.quaternion_to_rotation(quat_bc)

            # translation
            t_bc = np.array([
                T_base_camera.transform.translation.x,
                T_base_camera.transform.translation.y,
                T_base_camera.transform.translation.z
            ])

            # 2) Marker pose in camera frame
            t_cm = t # already numpy
            R_cm = R

            # 3) Compute marker in base frame:
            t_bm = R_bc @ t_cm + t_bc
            R_bm = R_bc @ R_cm

            # convert R_bm -> quaternion
            quat_base = self.rotation_matrix_to_quaternion(R_bm)

            # ---------------- Publish marker pose in base_link ----------------
            base_pose = PoseStamped()
            base_pose.header.frame_id = "base_link"
            base_pose.header.stamp = self.get_clock().now().to_msg()

            base_pose.pose.position.x = float(t_bm[0])
            base_pose.pose.position.y = float(t_bm[1])
            base_pose.pose.position.z = float(t_bm[2])

            base_pose.pose.orientation.x = quat_base[0]
            base_pose.pose.orientation.y = quat_base[1]
            base_pose.pose.orientation.z = quat_base[2]
            base_pose.pose.orientation.w = quat_base[3]

            self.marker_pub.publish(base_pose)

            self.get_logger().info(
                f"[Aruco in base_link] "
                f"x={t_bm[0]:.3f}, y={t_bm[1]:.3f}, z={t_bm[2]:.3f}"
            )

        except Exception as e:
            self.get_logger().warn(f"TF transform to base_link failed: {e}")

    def rotation_matrix_to_quaternion(self, R):
        """Convert a 3x3 rotation matrix to quaternion (x,y,z,w)"""
        q = np.zeros(4)
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2,1] - R[1,2]) * s
            q[1] = (R[0,2] - R[2,0]) * s
            q[2] = (R[1,0] - R[0,1]) * s
        else:
            if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
                q[3] = (R[2,1] - R[1,2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0,1] + R[1,0]) / s
                q[2] = (R[0,2] + R[2,0]) / s
            elif R[1,1] > R[2,2]:
                s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
                q[3] = (R[0,2] - R[2,0]) /s
                q[0] = (R[0,1] + R[1,0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1,2] + R[2,1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
                q[3] = (R[1,0] - R[0,1]) / s
                q[0] = (R[0,2] + R[2,0]) / s
                q[1] = (R[1,2] + R[2,1]) / s
                q[2] = 0.25 * s

        # Normalize quaternion (VERY IMPORTANT)
        q = q / np.linalg.norm(q)
        return q
    
    def quaternion_to_rotation(self, q):
        x, y, z, w = q
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w)],
            [2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w)],
            [2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y)]
        ])
        return R

def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
