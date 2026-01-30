#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy._rclpy_pybind11 import InvalidHandle

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, PoseArray
from sensor_msgs.msg import CompressedImage 
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf2_ros


def rotation_matrix_to_quaternion(R: np.ndarray):
    """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
    R = np.asarray(R, dtype=np.float64)
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

    return float(qx), float(qy), float(qz), float(qw)


class ArucoBoardTfAndPiecePoseNode(Node):
    """
    - 订阅已去畸变图像
    - 检测四角 ArUco，估计 board_frame 位姿
    - board_frame 原点定义为：左下(LD) marker 中心 + (dx, dy)
    - 发布 TF: camera_frame -> board_frame
    - 订阅棋子像素点 /piece_pixel (PoseArray: x=u, y=v)，发布 /piece_pose_board (PoseStamped, board_frame 下)
    - 发布 debug image
    """

    def __init__(self):
        super().__init__('aruco_board_tf_and_piece_pose_node')

        # ----------------------------
        # Params
        # ----------------------------
        self.declare_parameter('image_topic', '/camera/image_undistorted/compressed')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('camera_frame', 'camera_optical_link')
        self.declare_parameter('debug_image_topic', '/aruco_board/debug_image')

        self.declare_parameter('piece_pixel_topic', '/green_centers_array')
        self.declare_parameter('piece_pose_topic', '/piece_pose_board')

        self.declare_parameter('board_frame', 'board_frame')

        # corner_ids: [左上, 右上, 右下, 左下]
        self.declare_parameter('corner_ids', [0, 1, 3, 2])
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('hold_pose_sec', 10.0)

        # 物理尺寸（米）
        self.declare_parameter('marker_length', 0.016)   # 单个 ArUco 边长
        self.declare_parameter('board_length', 0.182)    # 四角 marker “中心”构成的正方形边长
        self.declare_parameter('axis_length', 0.06)     # 图像叠加显示坐标轴长度（米）

        # 原点偏移：默认 17.5mm, 17.5mm
        self.declare_parameter('origin_offset_from_ld', [0.008, 0.008])
         
        self.declare_parameter('grid_spacing_mm', 11.066667)
        self.declare_parameter('piece_grid_topic', '/piece_nearest_grid')

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value

        self.piece_pixel_topic = self.get_parameter('piece_pixel_topic').value
        self.piece_pose_topic = self.get_parameter('piece_pose_topic').value

        self.board_frame = self.get_parameter('board_frame').value

        self.corner_ids = list(self.get_parameter('corner_ids').value)
        self.aruco_dict_name = self.get_parameter('aruco_dict').value
        self.hold_pose_sec = float(self.get_parameter('hold_pose_sec').value)

        self.marker_length = float(self.get_parameter('marker_length').value)
        self.board_length = float(self.get_parameter('board_length').value)
        self.axis_length = float(self.get_parameter('axis_length').value)

        self.grid_spacing_mm = float(self.get_parameter('grid_spacing_mm').value)
        self.grid_spacing_m = self.grid_spacing_mm / 1000.0
        self.piece_grid_topic = self.get_parameter('piece_grid_topic').value


        off = self.get_parameter('origin_offset_from_ld').value
        self.origin_dx = float(off[0])
        self.origin_dy = float(off[1])

        # ----------------------------
        # OpenCV ArUco
        # ----------------------------
        self.aruco_dict = self._make_aruco_dict(self.aruco_dict_name)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        # 直接用“新原点”定义四个 marker 中心（你提到的 new_centers 方式）
        self.board = self._make_board_with_new_centers(
            corner_ids=self.corner_ids,
            marker_length=self.marker_length,
            board_length=self.board_length,
            origin_offset_from_ld=(self.origin_dx, self.origin_dy)
        )

        # ----------------------------
        # Camera model / state
        # ----------------------------
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.have_camera_info = False
        # Fixed parent frame for TF publishing
        self.camera_frame = self.camera_frame if self.camera_frame else "camera_optical_link"

        # 图像已去畸变：统一用 0 畸变
        self.zero_dist = np.zeros((5, 1), dtype=np.float64)

        # 最新 board pose (board->camera): X_c = R * X_b + t
        self.last_rvec = None
        self.last_tvec = None
        self.last_header = None
        self.last_valid_time = None

        # 最近一帧棋子像素点列表（u,v）
        self.last_piece_uvs = []
        self.piece_uv_buffer = []

        # ----------------------------
        # TF broadcaster + publishers
        # ----------------------------
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.pub_debug = self.create_publisher(Image, self.debug_image_topic, 10)
        self.pub_piece_pose = self.create_publisher(PoseStamped, self.piece_pose_topic, 10)
        self.pub_piece_grid = self.create_publisher(PointStamped, self.piece_grid_topic, 10)

        # ----------------------------
        # Subscribers
        # ----------------------------
        sensor_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub_info = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.on_camera_info,
            sensor_qos,
        )
        self.sub_img = self.create_subscription(CompressedImage, self.image_topic, self.on_image, 10)
        self.sub_piece = self.create_subscription(PoseArray, self.piece_pixel_topic, self.on_piece_pixel, 10)
        # Process at fixed rate (10 Hz) using latest frame
        self.latest_img_msg = None
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            f"Started.\n"
            f"  image_topic={self.image_topic}\n"
            f"  camera_info_topic={self.camera_info_topic}\n"
            f"  debug_image_topic={self.debug_image_topic}\n"
            f"  piece_pixel_topic={self.piece_pixel_topic}\n"
            f"  piece_pose_topic={self.piece_pose_topic}\n"
            f"  board_frame={self.board_frame}\n"
            f"  camera_frame={self.camera_frame}\n"
            f"  corner_ids(LU,RU,RD,LD)={self.corner_ids}\n"
            f"  marker_length={self.marker_length} m, board_length={self.board_length} m\n"
            f"  origin_offset_from_ld=(dx={self.origin_dx} m, dy={self.origin_dy} m)\n"
            f"  Image is assumed UNDISTORTED; using zero distortion in geometry."
        )

    def _make_aruco_dict(self, name: str):
        if not hasattr(cv2.aruco, name):
            raise ValueError(f"Unknown aruco_dict '{name}'. Example: DICT_4X4_50")
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))

    def _make_board_with_new_centers(self, corner_ids, marker_length, board_length, origin_offset_from_ld):
        """
        直接在“新board坐标系”下定义四个 marker 中心（new_centers）：

        - 先用一个直观坐标系：以左下(LD) marker 中心为 (0,0)
          LD: (0,0)
          RD: (L,0)
          RU: (L,L)
          LU: (0,L)

        - 然后把 board 原点放到：LD + (dx,dy)
          => 等效于所有中心点都减去 (dx,dy)

        corner_ids 顺序必须是 [左上, 右上, 右下, 左下]
        """
        L = float(board_length)
        half_m = float(marker_length) / 2.0
        dx, dy = origin_offset_from_ld

        # 以 LD 为参考的“中间坐标”
        ld = (0.0, 0.0, 0.0)
        rd = (L, 0.0, 0.0)
        ru = (L, L, 0.0)
        lu = (0.0, L, 0.0)

        # board 原点 = ld + (dx,dy)，所以中心点统一减去 (dx,dy)
        def shift(p):
            return (p[0] - dx, p[1] - dy, p[2])

        new_centers = [
            shift(lu),  # 左上
            shift(ru),  # 右上
            shift(rd),  # 右下
            shift(ld),  # 左下
        ]

        obj_points = []
        ids = []

        for i, marker_id in enumerate(corner_ids):
            cx, cy, cz = new_centers[i]
            # 角点顺序：左上、右上、右下、左下（与检测 corners 的顺序一致）
            pts = np.array([
                [cx - half_m, cy + half_m, cz],
                [cx + half_m, cy + half_m, cz],
                [cx + half_m, cy - half_m, cz],
                [cx - half_m, cy - half_m, cz],
            ], dtype=np.float32)

            obj_points.append(pts)
            ids.append(marker_id)

        obj_points = np.array(obj_points, dtype=np.float32)  # (nMarkers,4,3)
        ids = np.array(ids, dtype=np.int32)
        return cv2.aruco.Board(obj_points, self.aruco_dict, ids)

    def on_camera_info(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.have_camera_info = True
        # CameraInfo 一般只需要一次
        self.destroy_subscription(self.sub_info)
        self.get_logger().info(f"CameraInfo received. camera_frame='{self.camera_frame}'")

    def on_piece_pixel(self, msg: PoseArray):
        # 约定：pose.position.x=u, pose.position.y=v（像素）
        self.piece_uv_buffer = [
            (float(p.position.x), float(p.position.y)) for p in msg.poses
        ]
        self.last_piece_stamp = msg.header.stamp


    def _use_last_pose_or_clear(self, now, header, debug):
        if self.last_valid_time is not None and self.last_rvec is not None and self.last_tvec is not None:
            age_ns = (now - self.last_valid_time).nanoseconds
            if age_ns <= int(self.hold_pose_sec * 1e9):
                self._broadcast_camera_to_board_tf(header, self.last_rvec, self.last_tvec)
                if self.last_piece_uvs:
                    for idx, (u, v) in enumerate(self.last_piece_uvs):
                        cv2.circle(debug, (int(round(u)), int(round(v))), 6, (255, 255, 0), -1)
                        cv2.putText(debug, "piece_pixel", (int(round(u)) + 8, int(round(v)) - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)

                        pose = self._pixel_to_board_pose(
                            (u, v),
                            self.last_rvec,
                            self.last_tvec,
                            stamp=header.stamp,
                        )
                        if pose is not None:
                            self.pub_piece_pose.publish(pose)

                            X = pose.pose.position.x
                            Y = pose.pose.position.y

                            ij = self._nearest_grid_index(X, Y)
                            if ij is not None:
                                i, j = ij
                                self._publish_piece_grid(i, j, header.stamp, self.board_frame)

                                # 显示在 imshow（放左上角一行 + 放棋子像素点旁边一行）
                                y_row = 85 + (idx * 24)
                                cv2.putText(debug, f"nearest_grid=({i},{j})",
                                            (10, y_row), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                                cv2.putText(debug, f"({i},{j})", (int(round(u)) + 8, int(round(v)) + 22),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

                            cv2.putText(debug, f"board_xy=({X:.3f},{Y:.3f})m",
                                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

                cv2.drawFrameAxes(debug, self.camera_matrix, self.zero_dist, self.last_rvec, self.last_tvec, self.axis_length)
                origin_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
                origin_2d, _ = cv2.projectPoints(origin_3d, self.last_rvec, self.last_tvec, self.camera_matrix, self.zero_dist)
                ox, oy = origin_2d.reshape(-1).astype(int).tolist()
                cv2.circle(debug, (ox, oy), 5, (0, 255, 255), -1)
                cv2.putText(debug, "Board Origin(0,0)", (ox + 8, oy - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(debug, "Using last board pose", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 2, cv2.LINE_AA)
                self._publish_debug(debug, header)
                return True
        self.last_rvec = None
        self.last_tvec = None
        self.last_header = None
        self.last_valid_time = None
        self._publish_debug(debug, header)
        return False

    def on_image(self, msg: CompressedImage):
        # Cache latest frame; processing happens in timer callback
        self.latest_img_msg = msg

    def on_timer(self):
        msg = self.latest_img_msg
        if msg is None:
            return
        # consume the latest message so we don't reprocess the same frame
        self.latest_img_msg = None
        try:
            np_arr = np.frombuffer(msg.data,np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return
        
        if not self.have_camera_info:
            debug = frame.copy()
            cv2.putText(debug, "Waiting for camera_info", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            self._publish_debug(debug, msg.header)
            cv2.imshow('aruco_debug', debug)
            cv2.waitKey(1)
            return

        now = self.get_clock().now()
        if self.piece_uv_buffer:
            self.last_piece_uvs = self.piece_uv_buffer
            self.piece_uv_buffer = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        debug = frame.copy()

        if ids is None or len(ids) == 0:
            cv2.putText(debug, "No ArUco detected", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            self._use_last_pose_or_clear(now, msg.header, debug)
            cv2.imshow('aruco_debug', debug)
            cv2.waitKey(1)
            return

        cv2.aruco.drawDetectedMarkers(debug, corners, ids)
        required_ids = set(self.corner_ids)
        detected_ids = set(int(i) for i in ids.flatten())
        detected_required = required_ids.intersection(detected_ids)
        if len(detected_required) < 4:
            cv2.putText(debug, "Aruco <3 corners", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            self._use_last_pose_or_clear(now, msg.header, debug)
            cv2.imshow('aruco_debug', debug)
            cv2.waitKey(1)
            return

        rvec = np.zeros((3, 1), dtype=np.float64)
        tvec = np.zeros((3, 1), dtype=np.float64)

        # 图像已去畸变：用 zero_dist
        try:
            valid, rvec, tvec = cv2.aruco.estimatePoseBoard(
                corners, ids, self.board, self.camera_matrix, self.zero_dist, rvec, tvec
            )
        except Exception as e:
            self.get_logger().warn(f"estimatePoseBoard failed: {e}")
            cv2.putText(debug, "Pose estimate failed", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            self._use_last_pose_or_clear(now, msg.header, debug)
            cv2.imshow('aruco_debug', debug)
            cv2.waitKey(1)
            return

        if valid is None or valid <= 0:
            cv2.putText(debug, "Pose invalid", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            self._use_last_pose_or_clear(now, msg.header, debug)
            cv2.imshow('aruco_debug', debug)
            cv2.waitKey(1)
            return

        self.last_rvec = rvec
        self.last_tvec = tvec
        self.last_header = msg.header
        self.last_valid_time = now

        # 发布 TF: camera -> board
        self._broadcast_camera_to_board_tf(msg.header, rvec, tvec)

        # 可视化：画坐标轴（board 原点就是你定义的那个点）
        cv2.drawFrameAxes(debug, self.camera_matrix, self.zero_dist, rvec, tvec, self.axis_length)

        # 可视化：把 board 原点(0,0,0)投影出来画个点
        origin_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        origin_2d, _ = cv2.projectPoints(origin_3d, rvec, tvec, self.camera_matrix, self.zero_dist)
        ox, oy = origin_2d.reshape(-1).astype(int).tolist()
        cv2.circle(debug, (ox, oy), 5, (0, 255, 255), -1)
        cv2.putText(debug, "Board Origin(0,0)", (ox + 8, oy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.putText(debug, f"Detected IDs: {ids.flatten().tolist()}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 220, 50), 2, cv2.LINE_AA)

        # 如果已有棋子像素点：显示 + 发布棋子 Pose
        if self.last_piece_uvs:
            for idx, (u, v) in enumerate(self.last_piece_uvs):
                cv2.circle(debug, (int(round(u)), int(round(v))), 6, (255, 255, 0), -1)
                cv2.putText(debug, "piece_pixel", (int(round(u)) + 8, int(round(v)) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)

                pose = self._pixel_to_board_pose((u, v), rvec, tvec, stamp=msg.header.stamp)
                if pose is not None:
                    self.pub_piece_pose.publish(pose)

                    X = pose.pose.position.x
                    Y = pose.pose.position.y

                    ij = self._nearest_grid_index(X, Y)
                    if ij is not None:
                        i, j = ij
                        self._publish_piece_grid(i, j, msg.header.stamp, self.board_frame)

                        # 显示在 imshow（放左上角一行 + 放棋子像素点旁边一行）
                        y_row = 85 + (idx * 24)
                        cv2.putText(debug, f"nearest_grid=({i},{j})",
                                    (10, y_row), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                        cv2.putText(debug, f"({i},{j})", (int(round(u)) + 8, int(round(v)) + 22),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

                    cv2.putText(debug, f"board_xy=({X:.3f},{Y:.3f})m",
                                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

        self._publish_debug(debug, msg.header)
        cv2.imshow('aruco_debug', debug)
        cv2.waitKey(10)

    def _broadcast_camera_to_board_tf(self, header, rvec, tvec):
        """
        rvec/tvec: board->camera (X_c = R*X_b + t)
        TF 要发布 camera->board:
          R_cb = R^T
          t_cb = -R^T t
        """
        if not self.camera_frame:
            self.camera_frame = "camera_optical_link"

        R, _ = cv2.Rodrigues(rvec)
        R_cb = R.T
        t_cb = -R.T @ tvec  # 3x1

        qx, qy, qz, qw = rotation_matrix_to_quaternion(R_cb)

        ts = TransformStamped()
        ts.header.stamp = header.stamp
        ts.header.frame_id = self.camera_frame
        ts.child_frame_id = self.board_frame

        ts.transform.translation.x = float(t_cb[0, 0])
        ts.transform.translation.y = float(t_cb[1, 0])
        ts.transform.translation.z = float(t_cb[2, 0])

        ts.transform.rotation.x = qx
        ts.transform.rotation.y = qy
        ts.transform.rotation.z = qz
        ts.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(ts)
    
    def _nearest_grid_index(self, x_m: float, y_m: float):
        d = self.grid_spacing_m
        if d <= 0:
            return None
        i = int(np.round(x_m / d))
        j = int(np.round(y_m / d))
        # 如果你不允许负标号，可以取消下面两行注释
        # i = max(i, 0)
        # j = max(j, 0)
        return i, j

    def _publish_piece_grid(self, i: int, j: int, stamp, frame_id: str):
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.point.x = float(i)
        msg.point.y = float(j)
        msg.point.z = 0.0
        self.pub_piece_grid.publish(msg)

    def _pixel_to_board_pose(self, uv, rvec, tvec, stamp=None):
        """
        已去畸变图像：不 undistortPoints。
        像素点(u,v) -> 相机归一化射线 K^-1[u,v,1] -> 转到 board -> 与 Z=0 平面求交
        输出：board_frame 下 PoseStamped
        """
        u, v = float(uv[0]), float(uv[1])

        # 相机射线方向（相机坐标系）
        pix = np.array([[u], [v], [1.0]], dtype=np.float64)
        K_inv = np.linalg.inv(self.camera_matrix)
        d_c = K_inv @ pix  # 3x1

        # rvec->R (board->camera)
        R, _ = cv2.Rodrigues(rvec)

        # 转到 board：o_b + λ d_b
        o_b = -R.T @ tvec
        d_b = R.T @ d_c

        if abs(d_b[2, 0]) < 1e-9:
            return None

        lam = -o_b[2, 0] / d_b[2, 0]
        p_b = o_b + lam * d_b

        pose = PoseStamped()
        pose.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        pose.header.frame_id = self.board_frame

        pose.pose.position.x = float(p_b[0, 0])
        pose.pose.position.y = float(p_b[1, 0])
        pose.pose.position.z = 0.0

        # 不关心朝向就用单位四元数
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def _publish_debug(self, cv_img, header):
        out = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out.header = header
        self.pub_debug.publish(out)


def main():
    rclpy.init()
    node = ArucoBoardTfAndPiecePoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, InvalidHandle):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
