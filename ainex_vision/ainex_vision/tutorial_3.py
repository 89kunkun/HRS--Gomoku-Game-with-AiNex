#!/usr/bin/env python3
"""
TUM - ICS AiNex CameraSubscriberCompressed Demo for ROS 2 Jazzy
----------------------------------------
Subscribes to JPEG-compressed images and raw images on /camera_image/compressed and /camera_image,
shows frames with OpenCV, and displays CameraInfo.

Requires:
  sudo apt install python3-numpy python3-opencv

Msgs:
    sensor_msgs/CompressedImage
    sensor_msgs/CameraInfo
"""
from pathlib import Path
from cv2 import aruco

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
from rclpy.callback_groups import ReentrantCallbackGroup

TEMPLATE_PATH = Path("/home/hrs2025/Workspace/T2-template/ainex_vision/templateImg.png")
ROI_SAVE_PATH = Path("roi.png")
HIST_W, HIST_H = 360, 240

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.cb_group = ReentrantCallbackGroup()

        # QoS: Reliable to ensure camera_info is received
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribe compressed images
        self.sub_compressed = self.create_subscription(
            CompressedImage,
            'camera_image/compressed',
            self.image_callback_compressed,
            sensor_qos,
            callback_group=self.cb_group,
        )
        self.sub_compressed

        # Subscribe camera info
        self.sub_camerainfo = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.camera_info_callback,
            sensor_qos,
            callback_group=self.cb_group
        )
        self.sub_camerainfo

        # State variables
        self.camera_info_received = False
        self.frame = None
        self.paused = False

        # ========================================================
        # --------------------------------------------------------
        # Task 9
        self.prev_gray = None           # 上一帧（灰度）
        self.prev_pts = None            # 上一帧被跟踪的角点 Nx1x2 float32
        self.mask = None                # 轨迹画布，与图像同尺寸
        self.need_redetect = True       # 需要（重新）检测角点

        # 角点检测参数（可按需调整）
        self.feature_params = dict(
            maxCorners=20,
            qualityLevel=0.01,
            minDistance=30,
            blockSize=11
        )
        # LK 光流参数
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        # 少于这个数量则自动补点
        self.redetect_threshold = 15
        # ----------------------------------------------------------

        # ==== ROI/HSV ====
        self.current_roi = None
        self.roi_hist = None             # 第5步：ROI 的 Hue 直方图(已归一化到0..255)
        self.track_window = None         # 第6步：meanShift 矩形 (x,y,w,h)
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

        self.s_thresh = 50
        cv2.namedWindow("HSV Controls", cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("S_thresh", "HSV Controls", self.s_thresh, 255, self._on_s_thresh)
        
        # ---- ArUco (Task 10) ----
        self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # 与打印的标记一致
        self.aruco_params = aruco.DetectorParameters_create()
        self.marker_length_m = 0.04  # 按你的纸上真实边长(米)设置


        ## 从 CameraInfo 来的内参/畸变，收到后置上
        self.cam_K = None
        self.cam_D = None
        self.show_aruco = True
        
        # =========================================================

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
        
        # 始终更新（以防话题后续改变）
        self.cam_K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.cam_D = np.array(msg.d, dtype=np.float32).reshape(-1)

    def image_callback_compressed(self, msg: CompressedImage):
        try:
            # Decode the compressed image
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn('JPEG decode returned None')
                return
            # self.frame = frame

            self.frame_raw = frame
            self.frame = frame.copy()

        except Exception as exc:
            self.get_logger().error(f'Decode error in compressed image: {exc}')

    # ================================================================
    # ---------------- 饱和度滑条 ----------------
    def _on_s_thresh(self, val):
            self.s_thresh = int(val)
            if self.current_roi is not None:
                self.compute_and_show_hue_hist(self.current_roi)

    # ---------------- 保存当前帧为模板 ----------------
    def save_current_as_template(self):
        if self.frame is None:
            self.get_logger().warn("No camera frame yet.")
            return
        cv2.imwrite(str(TEMPLATE_PATH), self.frame)
        self.get_logger().info(f"Saved current frame -> {TEMPLATE_PATH}")

    # -------- step 3: select ROI --------
    def select_roi_from_template(self):
        if not TEMPLATE_PATH.exists():
            self.get_logger().warn(
                 f"{TEMPLATE_PATH} not found. Please make sure you already saved it in step 2."
            )
            return
        img = cv2.imread(str(TEMPLATE_PATH), cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f"Failed to read {TEMPLATE_PATH}")
            return

        self.get_logger().info(
             "Select ROI on the template window: drag to select, ENTER/SPACE to confirm, 'c' to cancel."
        )
        r = cv2.selectROI("Template (select ROI)", img, showCrosshair=True, fromCenter=False)
        cv2.destroyWindow("Template (select ROI)")

        x, y, w, h = map(int, r)
        if w == 0 or h == 0:
            self.get_logger().warn("Empty ROI, cancelled.")
            return
            
        roi = img[y:y+h, x:x+w].copy()
        self.current_roi = roi
        self.track_window = (x, y, w, h)  # 第6步初始窗口：沿用在模板上框的矩形
        cv2.imshow("ROI", roi)
        cv2.imwrite(str(ROI_SAVE_PATH), roi)
        self.get_logger().info(f"ROI saved to {ROI_SAVE_PATH}")

        # ====== 第5步所需：计算 ROI 的 Hue 直方图（带 S mask，归一化到 0..255）======
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h_roi, s_roi, _= cv2.split(hsv_roi)
        _, roi_mask = cv2.threshold(s_roi, self.s_thresh, 255, cv2.THRESH_BINARY)
        self.roi_hist = cv2.calcHist([h_roi], [0], roi_mask, [180], [0, 180])
        cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)

        self.compute_and_show_hue_hist(roi)

    # ---------------- 第4步：HSV + 阈值 + 直方图 ----------------
    def compute_and_show_hue_hist(self, roi_bgr):
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        h, s, _ = cv2.split(hsv)

        # 阈值遮掉低饱和像素
        _, mask = cv2.threshold(s, self.s_thresh, 255, cv2.THRESH_BINARY)

        # 计算 H 通道直方图
        hist = cv2.calcHist([h], [0], mask, [180], [0, 180])
        hist = cv2.normalize(hist, None, alpha=0, beta=HIST_H, norm_type=cv2.NORM_MINMAX)

        # 画直方图
        canvas = np.zeros((HIST_H, HIST_W, 3), dtype=np.uint8)
        bin_w = int(round(HIST_W / 180.0))
        for i in range(180):
            x1 = i * bin_w
            y1 = HIST_H - int(hist[i])
            x2 = (i + 1) * bin_w - 1
            y2 = HIST_H - 1
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (255, 255, 255), thickness=-1)

        # 展示：S 通道、mask、Hue 直方图
        cv2.imshow("S channel", s)
        # cv2.imshow("Mask (S > thresh)", mask)
        cv2.imshow("Hue Histogram", canvas)

    def compute_backprojection_step5(self):
        if self.frame is None or self.roi_hist is None:
            return None
        
        hsv = cv2.cvtColor(self.frame_raw, cv2.COLOR_BGR2HSV)
        h, s, _ = cv2.split(hsv)
        _, sat_mask = cv2.threshold(s, self.s_thresh, 255, cv2.THRESH_BINARY)

        backproj = cv2.calcBackProject([h], [0], self.roi_hist, [0, 180], scale=1)
        backproj = cv2.bitwise_and(backproj, sat_mask)
        backproj = cv2.GaussianBlur(backproj, (5, 5), 0)

        cv2.imshow("BackProjection (Step 5)", backproj)
        cv2.imshow("Saturation Mask", sat_mask)

        return backproj
    
    def mean_shift_step6(self, prob_map):
        if prob_map is None or self.track_window is None:
            return
        # 运行 meanShift：更新 self.track_window
        _, self.track_window = cv2.meanShift(prob_map, self.track_window, self.term_crit)
        x, y, w, h = self.track_window
        # 防越界裁剪
        H, W = self.frame.shape[:2]
        x = max(0, min(x, W-1)); y = max(0, min(y, H-1))
        w = max(1, min(w, W - x)); h = max(1, min(h, H - y))
        self.track_window = (x, y, w, h)
        # 在原彩色帧上画出矩形
        cv2.rectangle(self.frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

    def cam_shift_step7(self, prob_map):
        if prob_map is None or self.track_window is None:
            return
        # CamShift 返回旋转矩形 rot_rect 和更新后的 window
        rot_rect, self.track_window = cv2.CamShift(prob_map, self.track_window, self.term_crit)

        # rot_rect = ((cx, cy), (w, h), angle)
        box = cv2.boxPoints(rot_rect)
        box = np.int0(box)
        # 画旋转矩形（红色）
        cv2.polylines(self.frame, [box], isClosed=True, color=(0, 0, 255), thickness=2)

        # 可选：同时画个非旋转的包围框（绿色）
        x, y, w, h = self.track_window
        #cv2.rectangle(self.frame, (x, y), (x+w, y+h), (0, 255, 0), 1)

    # ------------------------------------------------------------------
    # ---------- Task 9: 准备/重置 ----------
    def grab_reference_and_detect(self):
        """把当前帧作为参考图，并在其上检测角点"""
        if self.frame is None:
            self.get_logger().warn("No camera frame yet.")
            return
        gray = cv2.cvtColor(self.frame_raw, cv2.COLOR_BGR2GRAY)
        pts = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
        if pts is None or len(pts) == 0:
            self.get_logger().warn("No features detected.")
            return

        self.prev_gray = gray
        self.prev_pts = pts.astype(np.float32)
        self.mask = np.zeros_like(self.frame)     # 轨迹画布清零
        self.need_redetect = False
        self.get_logger().info(f"Detected {len(self.prev_pts)} features.")

    def reset_tracks(self):
        self.prev_gray = None
        self.prev_pts = None
        self.mask = None
        self.need_redetect = True
        self.get_logger().info("Tracks reset.")

    # ---------- Task 9: 主处理（LK 光流 + 轨迹可视化） ----------
    def optical_flow_step(self):
        if self.frame is None:
            return

        frame_gray = cv2.cvtColor(self.frame_raw, cv2.COLOR_BGR2GRAY)

        # 需要（重新）检测角点
        if self.need_redetect or self.prev_gray is None or self.prev_pts is None:
            self.prev_gray = frame_gray
            self.prev_pts = cv2.goodFeaturesToTrack(self.prev_gray, mask=None, **self.feature_params)
            if self.prev_pts is None:
                return
            self.prev_pts = self.prev_pts.astype(np.float32)
            if self.mask is None or self.mask.shape != self.frame.shape:
                self.mask = np.zeros_like(self.frame)
            self.need_redetect = False

        # 计算光流
        next_pts, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, frame_gray, self.prev_pts, None, **self.lk_params
        )
        if next_pts is None or status is None:
            self.need_redetect = True
            return

        # 只保留成功跟踪的点，并 reshape 到 (N,2)
        st = status.flatten().astype(bool)
        good_next = next_pts[st].reshape(-1, 2)
        good_prev = self.prev_pts[st].reshape(-1, 2)

        # 如果没有有效点，则下次重检
        if good_next.size == 0:
            self.need_redetect = True
            return

        # 初始化轨迹画布
        if self.mask is None or self.mask.shape != self.frame.shape:
            self.mask = np.zeros_like(self.frame)
        else:
            self.mask = (self.mask * 0.95).astype(np.uint8)

        # 在 mask 上画线条（轨迹），在当前帧画点
        for (x2, y2), (x1, y1) in zip(good_next, good_prev):
            cv2.line(self.mask, (int(x2), int(y2)), (int(x1), int(y1)), (0, 255, 0), 2)
            cv2.circle(self.frame, (int(x2), int(y2)), 3, (0, 0, 255), -1)

        # 叠加显示：当前帧 + 轨迹
        output = cv2.add(self.frame, self.mask)
        cv2.imshow("Optical Flow", output)

        # 更新“上一帧/上一批点”
        self.prev_gray = frame_gray
        self.prev_pts = good_next.reshape(-1, 1, 2).astype(np.float32)

        # 若活点过少则触发下次重检
        if len(self.prev_pts) < self.redetect_threshold:
            self.need_redetect = True
    # -------------------------------------------------------------------

    def _inside_track_window(self, pt_xy):
        """判断某像素点是否位于当前跟踪窗口 track_window 内"""
        if self.track_window is None:
            return False
        x, y, w, h = self.track_window
        u, v = int(pt_xy[0]), int(pt_xy[1])
        return (x <= u < x + w) and (y <= v < y + h)

    def aruco_pose_step(self):
        """
        Task 10: 检测 ArUco 并在选中物体上显示 3D 坐标轴与位姿
        """
        if not self.show_aruco:
            return
        if self.frame_raw is None or self.cam_K is None or self.cam_D is None:
            return
        
        ids = None

        img = self.frame_raw.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # 调试输出
        if ids is None:
            self.get_logger().debug("No ArUco detected.")
        else:
            self.get_logger().debug(f"Detected ids: {ids.flatten().tolist()}")

        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(img, corners, ids)

            # ---- 位姿估计 ----
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length_m, self.cam_K, self.cam_D
            )
            # 类型强制为 float32，防止 drawFrameAxes 失败
            rvecs = np.asarray(rvecs, dtype=np.float32)
            tvecs = np.asarray(tvecs, dtype=np.float32)

            for i, mid in enumerate(ids.flatten()):
                center_px = corners[i][0].mean(axis=0)  # (u,v)
                # 放宽判断（边界多留 10 像素）
                if self.track_window is not None:
                    x, y, w, h = self.track_window
                    if not (x-10 <= center_px[0] <= x+w+10 and y-10 <= center_px[1] <= y+h+10):
                        continue

                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # ---- 画坐标轴 ----
                try:
                    cv2.drawFrameAxes(img, self.cam_K, self.cam_D, rvec, tvec, self.marker_length_m * 0.5)
                except Exception:
                    axis = np.float32([
                        [0,0,0],
                        [self.marker_length_m*0.5,0,0],
                        [0,self.marker_length_m*0.5,0],
                        [0,0,self.marker_length_m*0.5]
                    ])
                    pts, _ = cv2.projectPoints(axis, rvec, tvec, self.cam_K, self.cam_D)
                    pts = pts.reshape(-1,2).astype(int)
                    O, X, Y, Z = map(tuple, pts)
                    cv2.line(img, O, X, (0,0,255), 2)
                    cv2.line(img, O, Y, (0,255,0), 2)
                    cv2.line(img, O, Z, (255,0,0), 2)

                # ---- 打印/显示 3D 坐标 ----
                x, y, z = tvec.tolist()
                self.get_logger().info(f"[ArUco id={int(mid)}] t = [{x:.3f}, {y:.3f}, {z:.3f}] m")

                c = corners[i][0].mean(axis=0).astype(int)
                cv2.putText(img,
                            f"id {int(mid)}: [{x:.2f},{y:.2f},{z:.2f}] m",
                            (c[0]-70, c[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2, cv2.LINE_AA)

        cv2.imshow("3D marker position", img)
        print("Detected:", ids)

    # ===================================================================

    def process_key(self):
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            return False  # Quit
        if key == ord('p'):
            self.save_current_as_template()
        if key == ord('c'):
            self.show_compressed = True
            self.get_logger().info('Switched to compressed image')
        if key == ord('o'):
            self.select_roi_from_template()
        if key == ord('h'):
            if self.current_roi is not None:
                self.compute_and_show_hue_hist(self.current_roi)
            else:
                self.get_logger().warn("No ROI yet. Press 'o' to select one first.")
        if key == ord('m'):
            self.show_aruco = not self.show_aruco
            state = "ON" if self.show_aruco else "OFF"
            self.get_logger().info(f"ArUco window: {state}")
            if not self.show_aruco:
                try: cv2.destroyWindow("3D marker position")
                except: pass
        return True

    def display_loop(self):
        while rclpy.ok():
            if (self.frame is not None) and (not self.paused):
                # 第5步：若已有 ROI 直方图，则计算背投影
                self.frame[:] = self.frame_raw if self.frame_raw is not None else self.frame

                prob = self.compute_backprojection_step5() if self.roi_hist is not None else None
                if prob is not None and self.track_window is not None:
                    #self.mean_shift_step6(prob) # 第6步：在概率图上运行 meanShift 并画框
                    self.cam_shift_step7(prob)  # 第7步：在概率图上运行 camShift 并画框

                cv2.imshow('Camer Subscrber', self.frame)
                self.optical_flow_step()
                self.aruco_pose_step()

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