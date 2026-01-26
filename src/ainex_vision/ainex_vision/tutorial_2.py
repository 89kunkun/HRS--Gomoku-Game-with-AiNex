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

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PointStamped, Vector3Stamped

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

        # =====================================================
        # Publishers
        self.pub_centroid = self.create_publisher(PointStamped, '/blob/centroid_px', 10)
        self.pub_circle   = self.create_publisher(Vector3Stamped, '/hough/circle', 10)

        # Window names
        self.win_color = 'Camera Subscriber'
        self.win_gray = 'Greyscale image'
        self.win_bin  = 'Binary image'
        self.win_hsv  = 'Color extraction'
        self.win_blob = 'Blob extraction'
        self.win_circ = 'Circular shapes'

        # cv2.namedWindow(self.win_color, cv2.WINDOW_NORMAL)
        # cv2.namedWindow(self.win_gray, cv2.WINDOW_NORMAL)
        # cv2.namedWindow(self.win_bin, cv2.WINDOW_NORMAL)
        # cv2.namedWindow(self.win_hsv, cv2.WINDOW_NORMAL)

        for w in [self.win_color, self.win_gray, self.win_bin, self.win_hsv, self.win_blob, self.win_circ]:
            cv2.namedWindow(w, cv2.WINDOW_NORMAL)

        # Task 5 controls (binary threshold)
        self.thresh_value = 128 # 0~255
        self.use_inv = 0        
        cv2.createTrackbar('Threshold', self.win_bin, self.thresh_value, 255, self._on_trackbar_thresh)
        cv2.createTrackbar('Invert(0/1)', self.win_bin, self.use_inv, 1, self._on_trackbar_invert)

        # # Task 6 controls (HSV red extraction)
        # self.h1_min, self.h1_max = 0, 15
        # self.h2_min, self.h2_max = 160, 180
        # self.s_min,  self.v_min = 80, 70
        # cv2.createTrackbar('H1_min', self.win_hsv, self.h1_min, 180, lambda v: self._setattr('h1_min', v))
        # cv2.createTrackbar('H1_max', self.win_hsv, self.h1_max, 180, lambda v: self._setattr('h1_max', v))
        # cv2.createTrackbar('H2_min', self.win_hsv, self.h2_min, 180, lambda v: self._setattr('h2_min', v))
        # cv2.createTrackbar('H2_max', self.win_hsv, self.h2_max, 180, lambda v: self._setattr('h2_max', v))
        # cv2.createTrackbar('s_min',  self.win_hsv, self.h1_min, 255, lambda v: self._setattr('s_min',  v))
        # cv2.createTrackbar('v_min',  self.win_hsv, self.h1_min, 255, lambda v: self._setattr('v_min',  v))

        # Task 7: color selection 0=Red, 1=Green, 2=Blue
        self.color_sel = 0
        cv2.createTrackbar('Color(0-R, 1-G, 2-B)', self.win_hsv, self.color_sel, 2, self._on_color_change)

        self.defaults = {
            'red':   {'h1_min': 0,  'h1_max': 8, 'h2_min': 175, 'h2_max': 180, 's_min': 160, 'v_min': 70},
            'green': {'h1_min': 35, 'h1_max': 85, 'h2_min':   0, 'h2_max':   0, 's_min':  80, 'v_min': 50},
            'blue':  {'h1_min': 100,'h1_max': 140,'h2_min':   0, 'h2_max':   0, 's_min':  80, 'v_min': 50},
        }

        # Current HSV params(start with red)
        self._load_hsv_defaults('red')

        # Shared HSV trackbars (for the selected color)
        cv2.createTrackbar('H1_min', self.win_hsv, self.h1_min, 180, lambda v: self._setattr('h1_min', v))
        cv2.createTrackbar('H1_max', self.win_hsv, self.h1_max, 180, lambda v: self._setattr('h1_max', v))
        cv2.createTrackbar('H2_min', self.win_hsv, self.h2_min, 180, lambda v: self._setattr('h2_min', v))
        cv2.createTrackbar('H2_max', self.win_hsv, self.h2_max, 180, lambda v: self._setattr('h2_max', v))
        cv2.createTrackbar('S_min',  self.win_hsv, self.s_min,  255, lambda v: self._setattr('s_min',  v))
        cv2.createTrackbar('V_min',  self.win_hsv, self.v_min,  255, lambda v: self._setattr('v_min',  v))
        self._sync_hsv_trackbars()

        # Task 8: Morphological operation sliders
        self.kernel_size = 5
        self.erode_iter = 1
        self.dilate_iter = 2
        cv2.createTrackbar('Kernel(1-15)', self.win_blob, self.kernel_size, 15, lambda v: setattr(self, 'kernel_size', max(1, int(v))))
        cv2.createTrackbar('Erode(0-5)',   self.win_blob, self.erode_iter,  5,  lambda v: setattr(self, 'erode_iter',  max(0, int(v))))
        cv2.createTrackbar('Dilate(0-5)',  self.win_blob, self.dilate_iter, 5,  lambda v: setattr(self, 'dilate_iter', max(0, int(v))))

        # Task 9: Minimus area to accept a blob (px²)
        self.min_blob_area = 500
        cv2.createTrackbar('MinArea', self.win_blob, self.min_blob_area, 100000, lambda v: setattr(self, 'min_blob_area', int(v)))

        # Task 10: HoughCircles sliders
        # dp: inverse ratio of accumulator resolution to image resolution
        # minDist: minimum distance between circle centers
        # param1: higher Canny threshold (lower is param1/2)
        # param2: accumulator threshold for center detection
        # minRadius/maxRadius in pixels (0 means no bound for maxRadius)
        self.h_dp10     = 12   # dp=1.2
        self.h_minDist  = 30
        self.h_param1   = 100
        self.h_param2   = 60
        self.h_minR     = 5
        self.h_maxR     = 0
        cv2.createTrackbar('dp*10',    self.win_circ, self.h_dp10,    50, lambda v: setattr(self, 'h_dp10',    max(1, int(v))))
        cv2.createTrackbar('minDist',  self.win_circ, self.h_minDist, 200, lambda v: setattr(self, 'h_minDist', int(v)))
        cv2.createTrackbar('CannyHigh',self.win_circ, self.h_param1,  300, lambda v: setattr(self, 'h_param1',  int(v)))
        cv2.createTrackbar('AccThr',   self.win_circ, self.h_param2,  200, lambda v: setattr(self, 'h_param2',  int(v)))
        cv2.createTrackbar('minR',     self.win_circ, self.h_minR,    200, lambda v: setattr(self, 'h_minR',    int(v)))
        cv2.createTrackbar('maxR',     self.win_circ, self.h_maxR,    400, lambda v: setattr(self, 'h_maxR',    int(v)))

    # -----------------------------
    # Trackbar callbacks
    def _setattr(self, name, val):
        setattr(self, name, int(val))

    def _on_trackbar_thresh(self, val: int):
        self.thresh_value = int(val)
    
    def _on_trackbar_invert(self, val: int):
        self.use_inv = int(val)

    def _on_color_change(self, val:int):
        self.color_sel = int(val)
        color_name = ['red', 'green', 'blue'][self.color_sel]
        self._load_hsv_defaults(color_name)
        self._sync_hsv_trackbars()
        self.get_logger().info(f'Color extraction switched to: {color_name.upper()}')

    def _on_kernel(self, val): self.kernel_size = max(1, int(val))
    def _on_erode(self, val): self.erode_iter = max(0, int(val))
    def _on_dilate(self, val): self.dilate_iter = max(0, int(val))

    def _load_hsv_defaults(self, color_name: str):
        d = self.defaults[color_name]
        self.h1_min, self.h1_max = d['h1_min'], d['h1_max']
        self.h2_min, self.h2_max = d['h2_min'], d['h2_max']  # for red only; 0 otherwise
        self.s_min,  self.v_min  = d['s_min'],  d['v_min']

    def _sync_hsv_trackbars(self):
        cv2.setTrackbarPos('H1_min', self.win_hsv, self.h1_min)
        cv2.setTrackbarPos('H1_max', self.win_hsv, self.h1_max)
        cv2.setTrackbarPos('H2_min', self.win_hsv, self.h2_min)
        cv2.setTrackbarPos('H2_max', self.win_hsv, self.h2_max)
        cv2.setTrackbarPos('S_min',  self.win_hsv, self.s_min)
        cv2.setTrackbarPos('V_min',  self.win_hsv, self.v_min)
    # -----------------------------

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

    def image_callback_compressed(self, msg: CompressedImage):
        try:
            # Decode the compressed image
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn('JPEG decode returned None')
                return
            self.frame = frame
        except Exception as exc:
            self.get_logger().error(f'Decode error in compressed image: {exc}')

    def process_key(self):
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            return False  # Quit
        if key == ord('c'):
            self.show_compressed = True
            self.get_logger().info('Switched to compressed image')
        return True

    # def _hsv_red_mask(self, bgr):
    #     hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    #     # lower red mask1
    #     lower1 = np.array([min(self.h1_min, self.h1_max), self.s_min, self.v_min], dtype=np.uint8)
    #     upper1 = np.array([max(self.h1_min, self.h1_max), 255, 255], dtype=np.uint8)
    #     mask1 = cv2.inRange(hsv, lower1, upper1)
    #     # upper red mask2
    #     lower2 = np.array([min(self.h2_min, self.h2_max), self.s_min, self.v_min], dtype=np.uint8)
    #     upper2 = np.array([max(self.h2_min, self.h2_max), 180, 255], dtype=np.uint8)
    #     mask2 = cv2.inRange(hsv, lower2, upper2)

    #     mask = cv2.bitwise_or(mask1, mask2)
    #     return mask

    # ----------------------------------------------------------
    # HSV masks (R/G/B in same window)
    def _hsv_color_mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        color = ['red', 'green', 'blue'][self.color_sel]

        if color == 'red':
            # Red needs two hue bands
            lower1 = np.array([min(self.h1_min, self.h1_max), self.s_min, self.v_min], dtype=np.uint8)
            upper1 = np.array([max(self.h1_min, self.h1_max), 255, 255], dtype=np.uint8)
            mask1 = cv2.inRange(hsv, lower1, upper1)

            lower2 = np.array([min(self.h2_min, self.h2_max), self.s_min, self.v_min], dtype=np.uint8)
            upper2 = np.array([max(self.h2_min, self.h2_max), 180, 255], dtype=np.uint8)
            mask2 = cv2.inRange(hsv, lower2, upper2)

            mask = cv2.bitwise_or(mask1, mask2)

        else:
            # Green or Blue use single band on H1
            h_min = min(self.h1_min, self.h1_max)
            h_max = max(self.h1_min, self.h1_max)
            lower = np.array([h_min, self.s_min, self.v_min], dtype=np.uint8)
            upper = np.array([h_max, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)

        return mask
    
    # Apply morphology to form blobs
    def _blobify(self, mask):
        k = max(1, self.kernel_size)
        if k % 2 == 0: # kernel must be odd
            k += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        result = mask.copy()
        if self.erode_iter > 0:
            result = cv2.erode(result, kernel, iterations=self.erode_iter)
        if self.dilate_iter > 0:
            result = cv2.dilate(result, kernel, iterations=self.dilate_iter)
        return result

    # Return (cx, cy, contour) for largest blob above min area; else None
    def _largest_blob_centroid(self, mask):
        contours, _= cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Largest by area
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.min_blob_area:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return (cx, cy, largest)

    # HoughCircles
    def _detect_circles(self, gray):
        # Pre-processing: slight blur helps
        blur = cv2.medianBlur(gray, 5)
        dp = max(0.1, self.h_dp10 / 10.0)
        minDist  = max(1, self.h_minDist)
        param1   = max(1, self.h_param1)
        param2   = max(1, self.h_param2)
        minR     = max(0, self.h_minR)
        maxR     = max(0, self.h_maxR)
        circles = cv2.HoughCircles(
            blur,
            cv2.HOUGH_GRADIENT,
            dp=dp,
            minDist=minDist,
            param1=param1,
            param2=param2,
            minRadius=minR,
            maxRadius=maxR if maxR > 0 else 0
        )
        if circles is None:
            return []
        circles = np.round(circles[0, :]).astype(int)  # Nx3: (x,y,r)
        return circles.tolist()
    # ----------------------------------------------------------

    def display_loop(self):
        while rclpy.ok():
            if self.frame is not None:
                # Display the compressed image
                cv2.imshow(self.win_color, self.frame)

                # Greyscale
                gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
                cv2.imshow(self.win_gray, gray)

                # Binary
                thresh_type = cv2.THRESH_BINARY_INV if self.use_inv else cv2.THRESH_BINARY
                _, binary = cv2.threshold(gray, self.thresh_value, 255, thresh_type)
                cv2.imshow(self.win_bin, binary)

                # Color extraction - red mask
                # red_mask = self._hsv_red_mask(self.frame)
                # cv2.imshow(self.win_hsv, red_mask)
                color_mask = self._hsv_color_mask(self.frame)
                cv2.imshow(self.win_hsv, color_mask)

                # Blob extraction 
                blob = self._blobify(color_mask)
                cv2.imshow(self.win_blob, blob)

                # Task 9: find largest blob, draw centroid, publish
                blob_vis = cv2.cvtColor(blob, cv2.COLOR_GRAY2BGR)  # for colored overlay
                res = self._largest_blob_centroid(blob)
                if res is not None:
                    cx, cy, contour = res
                    # draw contour (green) & centroid (red)
                    cv2.drawContours(blob_vis, [contour], -1, (0, 255, 0), 2)
                    cv2.circle(blob_vis, (cx, cy), 4, (0, 0, 255), -1)

                    # publish centroid as PointStamped (pixel coords)
                    msg = PointStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'image'  # pixels in the image plane
                    msg.point.x = float(cx)
                    msg.point.y = float(cy)
                    msg.point.z = 0.0
                    self.pub_centroid.publish(msg)
                else:
                    pass
                cv2.imshow(self.win_blob, blob_vis)

                # Task 10: Circular shapes (Hough)
                circles = self._detect_circles(gray)
                circ_vis = self.frame.copy()
                if circles:
                    # Draw all circles (green), mark largest (red)
                    largest_circle = max(circles, key=lambda c: c[2])
                    for (x, y, r) in circles:
                        cv2.circle(circ_vis, (x, y), r, (0, 255, 0), 2)
                        cv2.circle(circ_vis, (x, y), 2, (0, 255, 0), -1)
                    lx, ly, lr = largest_circle
                    cv2.circle(circ_vis, (lx, ly), lr, (0, 0, 255), 2)
                    cv2.circle(circ_vis, (lx, ly), 3, (0, 0, 255), -1)

                    # Publish largest circle center & radius (Vector3Stamped)
                    msg = Vector3Stamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'image'
                    msg.vector.x = float(lx)
                    msg.vector.y = float(ly)
                    msg.vector.z = float(lr)  # radius in pixels
                    self.pub_circle.publish(msg)
                cv2.imshow(self.win_circ, circ_vis)

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
