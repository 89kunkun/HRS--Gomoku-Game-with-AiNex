# s
# import rclpy
# from rclpy.node import Node

# import cv2
# import numpy as np
# from cv_bridge import CvBridge

# from sensor_msgs.msg import CompressedImage
# from geometry_msgs.msg import PoseArray, Pose


# class RedBlobDetectorCompressed(Node):
#     def __init__(self):
#         super().__init__('red_blob_detector_compressed')

#         self.bridge = CvBridge()

#         # ---------------- parameters ----------------
#         self.declare_parameter('image_topic', '/camera/image_undistorted/compressed')
#         self.declare_parameter('points_topic', '/red_centers_array')
#         self.declare_parameter('min_area', 50)
#         self.declare_parameter('debug_view', True)
#         self.declare_parameter('track_max_dist', 1.0)
#         self.declare_parameter('track_hold_sec', 1.0)
#         self.declare_parameter('smooth_alpha', 0.2)

#         # HSV threshold for RED (two ranges because H wraps around 0/180)
#         # Good start for your red pieces:
#         self.declare_parameter('h1_low', 0)
#         self.declare_parameter('h1_high', 15)
#         self.declare_parameter('h2_low', 165)
#         self.declare_parameter('h2_high', 180)
#         self.declare_parameter('s_low', 80)
#         self.declare_parameter('s_high', 255)
#         self.declare_parameter('v_low', 50)
#         self.declare_parameter('v_high', 255)

#         # morphology
#         self.declare_parameter('kernel_size', 3)
#         self.declare_parameter('open_iters', 1)
#         self.declare_parameter('close_iters', 0)

#         # ---------------- read params ----------------
#         self.image_topic = self.get_parameter('image_topic').value
#         self.points_topic = self.get_parameter('points_topic').value
#         self.min_area = int(self.get_parameter('min_area').value)
#         self.debug_view = bool(self.get_parameter('debug_view').value)
#         self.smooth_alpha = float(self.get_parameter('smooth_alpha').value)

#         # ---------------- ROS I/O ----------------
#         self.sub = self.create_subscription(
#             CompressedImage, self.image_topic, self.on_image, 10
#         )
#         self.point_pub = self.create_publisher(
#             PoseArray, self.points_topic, 10
#         )

#         # track state
#         self.tracks = {}
#         self.next_track_id = 1

#         self.get_logger().info(
#             f"Sub: {self.image_topic} | Pub: {self.points_topic} (PoseArray: x=u, y=v)"
#         )

#     # ------------------------------------------------
#     def _get_hsv_bounds(self):
#         s_low = int(self.get_parameter('s_low').value)
#         s_high = int(self.get_parameter('s_high').value)
#         v_low = int(self.get_parameter('v_low').value)
#         v_high = int(self.get_parameter('v_high').value)

#         lower1 = np.array([
#             int(self.get_parameter('h1_low').value),
#             s_low, v_low
#         ], dtype=np.uint8)
#         upper1 = np.array([
#             int(self.get_parameter('h1_high').value),
#             s_high, v_high
#         ], dtype=np.uint8)

#         lower2 = np.array([
#             int(self.get_parameter('h2_low').value),
#             s_low, v_low
#         ], dtype=np.uint8)
#         upper2 = np.array([
#             int(self.get_parameter('h2_high').value),
#             s_high, v_high
#         ], dtype=np.uint8)

#         return (lower1, upper1), (lower2, upper2)

#     # ------------------------------------------------
#     def find_red_centers_and_mask(self, bgr):
#         hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
#         (l1, u1), (l2, u2) = self._get_hsv_bounds()

#         # 1) threshold (two ranges)
#         mask1 = cv2.inRange(hsv, l1, u1)
#         mask2 = cv2.inRange(hsv, l2, u2)
#         mask = cv2.bitwise_or(mask1, mask2)

#         # 2) morphology
#         ksize = int(self.get_parameter('kernel_size').value)
#         if ksize % 2 == 0:
#             ksize += 1
#         k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))

#         mask = cv2.morphologyEx(
#             mask, cv2.MORPH_OPEN, k,
#             iterations=int(self.get_parameter('open_iters').value)
#         )
#         mask = cv2.morphologyEx(
#             mask, cv2.MORPH_CLOSE, k,
#             iterations=int(self.get_parameter('close_iters').value)
#         )

#         # 3) connected components
#         num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

#         centers = []
#         for label in range(1, num_labels):
#             area = int(stats[label, cv2.CC_STAT_AREA])
#             if area < self.min_area:
#                 continue
#             cx, cy = centroids[label]
#             centers.append((float(cx), float(cy), area))

#         return centers, mask

#     # ------------------------------------------------
#     def _prune_tracks(self, now):
#         hold_ns = int(float(self.get_parameter('track_hold_sec').value) * 1e9)
#         to_remove = []
#         for track_id, track in self.tracks.items():
#             if (now - track['last_time']).nanoseconds > hold_ns:
#                 to_remove.append(track_id)
#         for track_id in to_remove:
#             del self.tracks[track_id]

#     def _update_tracks(self, centers, now):
#         max_dist = float(self.get_parameter('track_max_dist').value)
#         alpha = float(self.get_parameter('smooth_alpha').value)
#         if alpha < 0.0:
#             alpha = 0.0
#         elif alpha > 1.0:
#             alpha = 1.0
#         unmatched_tracks = set(self.tracks.keys())

#         for cx, cy, area in centers:
#             best_id = None
#             best_dist = None
#             for track_id in unmatched_tracks:
#                 tx, ty = self.tracks[track_id]['cx'], self.tracks[track_id]['cy']
#                 dist = np.hypot(cx - tx, cy - ty)
#                 if best_dist is None or dist < best_dist:
#                     best_dist = dist
#                     best_id = track_id

#             if best_id is not None and best_dist <= max_dist:
#                 fx = alpha * cx + (1.0 - alpha) * self.tracks[best_id].get('fx', cx)
#                 fy = alpha * cy + (1.0 - alpha) * self.tracks[best_id].get('fy', cy)
#                 self.tracks[best_id].update({
#                     'cx': fx,
#                     'cy': fy,
#                     'fx': fx,
#                     'fy': fy,
#                     'area': area,
#                     'last_time': now,
#                 })
#                 unmatched_tracks.remove(best_id)
#             else:
#                 self.tracks[self.next_track_id] = {
#                     'cx': cx,
#                     'cy': cy,
#                     'fx': cx,
#                     'fy': cy,
#                     'area': area,
#                     'last_time': now,
#                 }
#                 self.next_track_id += 1

#         self._prune_tracks(now)
#         return [(t.get('fx', t['cx']), t.get('fy', t['cy']), t['area']) for t in self.tracks.values()]

#     # ------------------------------------------------
#     def on_image(self, msg: CompressedImage):
#         # CompressedImage -> OpenCV
#         frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         centers, mask = self.find_red_centers_and_mask(frame)
#         now = self.get_clock().now()
#         tracked_centers = self._update_tracks(centers, now)

#         # ===== publish PoseArray (pixel coords) =====
#         out = PoseArray()
#         out.header = msg.header
#         for cx, cy, area in tracked_centers:
#             pose = Pose()
#             pose.position.x = float(cx)  # u
#             pose.position.y = float(cy)  # v
#             pose.position.z = 0.0
#             pose.orientation.w = 1.0
#             out.poses.append(pose)
#         self.point_pub.publish(out)

#         # ===== visualization =====
#         if self.debug_view:
#             vis = frame.copy()
#             for cx, cy, area in tracked_centers:
#                 u, v = int(round(cx)), int(round(cy))
#                 cv2.circle(vis, (u, v), 6, (0, 0, 255), -1)
#                 cv2.putText(
#                     vis, f"({u},{v}) a={area}",
#                     (u + 8, v - 8),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.6, (0, 0, 255), 2
#                 )

#             cv2.imshow("red_mask", mask)
#             cv2.imshow("red_vis", vis)
#             cv2.waitKey(1)


# def main():
#     rclpy.init()
#     node = RedBlobDetectorCompressed()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node

# import cv2
# import numpy as np
# from cv_bridge import CvBridge

# from sensor_msgs.msg import CompressedImage
# from geometry_msgs.msg import PoseArray, Pose


# class GreenBlobDetectorCompressed(Node):
#     def __init__(self):
#         super().__init__('green_blob_detector_compressed')

#         self.bridge = CvBridge()

#         # ---------------- parameters ----------------
#         self.declare_parameter('image_topic', '/camera/image_undistorted/compressed')
#         self.declare_parameter('points_topic', '/green_centers_array')
#         self.declare_parameter('min_area', 50)
#         self.declare_parameter('debug_view', True)
#         self.declare_parameter('track_max_dist', 1.0)
#         self.declare_parameter('track_hold_sec', 1.0)
#         self.declare_parameter('smooth_alpha', 0.2)

#         # HSV threshold for GREEN (single range)
#         # Good start for your hand-drawn green:
#         self.declare_parameter('h_low', 30)
#         self.declare_parameter('h_high', 102)
#         self.declare_parameter('s_low', 70)
#         self.declare_parameter('s_high', 255)
#         self.declare_parameter('v_low', 30)
#         self.declare_parameter('v_high', 255)

#         # morphology
#         self.declare_parameter('kernel_size', 3)
#         self.declare_parameter('open_iters', 1)
#         self.declare_parameter('close_iters', 0)

#         # ---------------- read params ----------------
#         self.image_topic = self.get_parameter('image_topic').value
#         self.points_topic = self.get_parameter('points_topic').value
#         self.min_area = int(self.get_parameter('min_area').value)
#         self.debug_view = bool(self.get_parameter('debug_view').value)
#         self.smooth_alpha = float(self.get_parameter('smooth_alpha').value)

#         # ---------------- ROS I/O ----------------
#         self.sub = self.create_subscription(
#             CompressedImage, self.image_topic, self.on_image, 10
#         )
#         self.point_pub = self.create_publisher(
#             PoseArray, self.points_topic, 10
#         )

#         # track state
#         self.tracks = {}
#         self.next_track_id = 1

#         self.get_logger().info(
#             f"Sub: {self.image_topic} | Pub: {self.points_topic} (PoseArray: x=u, y=v)"
#         )

#     # ------------------------------------------------
#     def _get_hsv_bounds(self):
#         h_low = int(self.get_parameter('h_low').value)
#         h_high = int(self.get_parameter('h_high').value)
#         s_low = int(self.get_parameter('s_low').value)
#         s_high = int(self.get_parameter('s_high').value)
#         v_low = int(self.get_parameter('v_low').value)
#         v_high = int(self.get_parameter('v_high').value)

#         lower = np.array([h_low, s_low, v_low], dtype=np.uint8)
#         upper = np.array([h_high, s_high, v_high], dtype=np.uint8)
#         return lower, upper

#     # ------------------------------------------------
#     def find_green_centers_and_mask(self, bgr):
#         hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
#         lower, upper = self._get_hsv_bounds()

#         # 1) threshold (single range)
#         mask = cv2.inRange(hsv, lower, upper)

#         # 2) morphology
#         ksize = int(self.get_parameter('kernel_size').value)
#         if ksize % 2 == 0:
#             ksize += 1
#         k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))

#         mask = cv2.morphologyEx(
#             mask, cv2.MORPH_OPEN, k,
#             iterations=int(self.get_parameter('open_iters').value)
#         )
#         mask = cv2.morphologyEx(
#             mask, cv2.MORPH_CLOSE, k,
#             iterations=int(self.get_parameter('close_iters').value)
#         )

#         # 3) connected components
#         num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

#         centers = []
#         for label in range(1, num_labels):
#             area = int(stats[label, cv2.CC_STAT_AREA])
#             if area < self.min_area:
#                 continue
#             cx, cy = centroids[label]
#             centers.append((float(cx), float(cy), area))

#         return centers, mask

#     # ------------------------------------------------
#     def _prune_tracks(self, now):
#         hold_ns = int(float(self.get_parameter('track_hold_sec').value) * 1e9)
#         to_remove = []
#         for track_id, track in self.tracks.items():
#             if (now - track['last_time']).nanoseconds > hold_ns:
#                 to_remove.append(track_id)
#         for track_id in to_remove:
#             del self.tracks[track_id]

#     def _update_tracks(self, centers, now):
#         max_dist = float(self.get_parameter('track_max_dist').value)
#         alpha = float(self.get_parameter('smooth_alpha').value)
#         if alpha < 0.0:
#             alpha = 0.0
#         elif alpha > 1.0:
#             alpha = 1.0
#         unmatched_tracks = set(self.tracks.keys())

#         for cx, cy, area in centers:
#             best_id = None
#             best_dist = None
#             for track_id in unmatched_tracks:
#                 tx, ty = self.tracks[track_id]['cx'], self.tracks[track_id]['cy']
#                 dist = np.hypot(cx - tx, cy - ty)
#                 if best_dist is None or dist < best_dist:
#                     best_dist = dist
#                     best_id = track_id

#             if best_id is not None and best_dist <= max_dist:
#                 fx = alpha * cx + (1.0 - alpha) * self.tracks[best_id].get('fx', cx)
#                 fy = alpha * cy + (1.0 - alpha) * self.tracks[best_id].get('fy', cy)
#                 self.tracks[best_id].update({
#                     'cx': fx,
#                     'cy': fy,
#                     'fx': fx,
#                     'fy': fy,
#                     'area': area,
#                     'last_time': now,
#                 })
#                 unmatched_tracks.remove(best_id)
#             else:
#                 self.tracks[self.next_track_id] = {
#                     'cx': cx,
#                     'cy': cy,
#                     'fx': cx,
#                     'fy': cy,
#                     'area': area,
#                     'last_time': now,
#                 }
#                 self.next_track_id += 1

#         self._prune_tracks(now)
#         return [(t.get('fx', t['cx']), t.get('fy', t['cy']), t['area']) for t in self.tracks.values()]

#     # ------------------------------------------------
#     def on_image(self, msg: CompressedImage):
#         # CompressedImage -> OpenCV
#         frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         centers, mask = self.find_green_centers_and_mask(frame)
#         now = self.get_clock().now()
#         tracked_centers = self._update_tracks(centers, now)

#         # ===== publish PoseArray (pixel coords) =====
#         out = PoseArray()
#         out.header = msg.header
#         for cx, cy, area in tracked_centers:
#             pose = Pose()
#             pose.position.x = float(cx)  # u
#             pose.position.y = float(cy)  # v
#             pose.position.z = 0.0
#             pose.orientation.w = 1.0
#             out.poses.append(pose)
#         self.point_pub.publish(out)

#         # ===== visualization =====
#         if self.debug_view:
#             vis = frame.copy()
#             for cx, cy, area in tracked_centers:
#                 u, v = int(round(cx)), int(round(cy))
#                 cv2.circle(vis, (u, v), 6, (0, 0, 255), -1)
#                 cv2.putText(
#                     vis, f"({u},{v}) a={area}",
#                     (u + 8, v - 8),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.6, (0, 0, 255), 2
#                 )

#             cv2.imshow("green_mask", mask)
#             cv2.imshow("green_vis", vis)
#             cv2.waitKey(1)


# def main():
#     rclpy.init()
#     node = GreenBlobDetectorCompressed()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()
# !/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray, Pose


class GreenBlobDetectorCompressed(Node):
    def __init__(self):
        super().__init__('green_blob_detector_compressed')

        self.bridge = CvBridge()

        # ---------------- parameters ----------------
        self.declare_parameter('image_topic', '/camera/image_undistorted/compressed')
        self.declare_parameter('points_topic', '/green_centers_array')
        self.declare_parameter('min_area', 50)
        self.declare_parameter('debug_view', True)
        self.declare_parameter('track_max_dist', 1.0)
        self.declare_parameter('track_hold_sec', 1.0)
        self.declare_parameter('smooth_alpha', 0.2)

        # HSV threshold for GREEN (single range) - defaults (can be tuned live)
        self.declare_parameter('h_low', 30)
        self.declare_parameter('h_high', 102)
        self.declare_parameter('s_low', 70)
        self.declare_parameter('s_high', 255)
        self.declare_parameter('v_low', 30)
        self.declare_parameter('v_high', 255)

        # morphology
        self.declare_parameter('kernel_size', 3)
        self.declare_parameter('open_iters', 1)
        self.declare_parameter('close_iters', 0)

        # ---------------- read params ----------------
        self.image_topic = self.get_parameter('image_topic').value
        self.points_topic = self.get_parameter('points_topic').value
        self.min_area = int(self.get_parameter('min_area').value)
        self.debug_view = bool(self.get_parameter('debug_view').value)
        self.smooth_alpha = float(self.get_parameter('smooth_alpha').value)

        # ---------------- ROS I/O ----------------
        self.sub = self.create_subscription(
            CompressedImage, self.image_topic, self.on_image, 10
        )
        self.point_pub = self.create_publisher(
            PoseArray, self.points_topic, 10
        )

        # track state
        self.tracks = {}
        self.next_track_id = 1

        # ---------- HSV tuner (trackbars) ----------
        self.use_hsv_tuner = self.debug_view  # only when GUI enabled
        if self.use_hsv_tuner:
            cv2.namedWindow("HSV_Tuner", cv2.WINDOW_NORMAL)

            cv2.createTrackbar("H_low",  "HSV_Tuner", int(self.get_parameter('h_low').value), 180, lambda x: None)
            cv2.createTrackbar("H_high", "HSV_Tuner", int(self.get_parameter('h_high').value), 180, lambda x: None)

            cv2.createTrackbar("S_low",  "HSV_Tuner", int(self.get_parameter('s_low').value), 255, lambda x: None)
            cv2.createTrackbar("S_high", "HSV_Tuner", int(self.get_parameter('s_high').value), 255, lambda x: None)

            cv2.createTrackbar("V_low",  "HSV_Tuner", int(self.get_parameter('v_low').value), 255, lambda x: None)
            cv2.createTrackbar("V_high", "HSV_Tuner", int(self.get_parameter('v_high').value), 255, lambda x: None)

        self.get_logger().info(
            f"Sub: {self.image_topic} | Pub: {self.points_topic} (PoseArray: x=u, y=v) | "
            f"HSV tuner: {'ON' if self.use_hsv_tuner else 'OFF'}"
        )

    # ------------------------------------------------
    def _get_hsv_bounds(self):
        if self.use_hsv_tuner:
            h_low = cv2.getTrackbarPos("H_low", "HSV_Tuner")
            h_high = cv2.getTrackbarPos("H_high", "HSV_Tuner")
            s_low = cv2.getTrackbarPos("S_low", "HSV_Tuner")
            s_high = cv2.getTrackbarPos("S_high", "HSV_Tuner")
            v_low = cv2.getTrackbarPos("V_low", "HSV_Tuner")
            v_high = cv2.getTrackbarPos("V_high", "HSV_Tuner")
        else:
            h_low = int(self.get_parameter('h_low').value)
            h_high = int(self.get_parameter('h_high').value)
            s_low = int(self.get_parameter('s_low').value)
            s_high = int(self.get_parameter('s_high').value)
            v_low = int(self.get_parameter('v_low').value)
            v_high = int(self.get_parameter('v_high').value)

        # guard: ensure low <= high
        if h_low > h_high:
            h_low, h_high = h_high, h_low
        if s_low > s_high:
            s_low, s_high = s_high, s_low
        if v_low > v_high:
            v_low, v_high = v_high, v_low

        lower = np.array([h_low, s_low, v_low], dtype=np.uint8)
        upper = np.array([h_high, s_high, v_high], dtype=np.uint8)
        return lower, upper

    # ------------------------------------------------
    def find_green_centers_and_mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower, upper = self._get_hsv_bounds()

        # 1) threshold (single range)
        mask = cv2.inRange(hsv, lower, upper)

        # 2) morphology
        ksize = int(self.get_parameter('kernel_size').value)
        if ksize < 1:
            ksize = 1
        if ksize % 2 == 0:
            ksize += 1
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))

        open_iters = int(self.get_parameter('open_iters').value)
        close_iters = int(self.get_parameter('close_iters').value)

        if open_iters > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=open_iters)
        if close_iters > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=close_iters)

        # 3) connected components
        num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

        centers = []
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.min_area:
                continue
            cx, cy = centroids[label]
            centers.append((float(cx), float(cy), area))

        return centers, mask

    # ------------------------------------------------
    def _prune_tracks(self, now):
        hold_ns = int(float(self.get_parameter('track_hold_sec').value) * 1e9)
        to_remove = []
        for track_id, track in self.tracks.items():
            if (now - track['last_time']).nanoseconds > hold_ns:
                to_remove.append(track_id)
        for track_id in to_remove:
            del self.tracks[track_id]

    def _update_tracks(self, centers, now):
        max_dist = float(self.get_parameter('track_max_dist').value)
        alpha = float(self.get_parameter('smooth_alpha').value)
        alpha = max(0.0, min(1.0, alpha))

        unmatched_tracks = set(self.tracks.keys())

        for cx, cy, area in centers:
            best_id = None
            best_dist = None
            for track_id in unmatched_tracks:
                tx, ty = self.tracks[track_id]['cx'], self.tracks[track_id]['cy']
                dist = np.hypot(cx - tx, cy - ty)
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_id = track_id

            if best_id is not None and best_dist <= max_dist:
                fx_prev = self.tracks[best_id].get('fx', cx)
                fy_prev = self.tracks[best_id].get('fy', cy)
                fx = alpha * cx + (1.0 - alpha) * fx_prev
                fy = alpha * cy + (1.0 - alpha) * fy_prev
                self.tracks[best_id].update({
                    'cx': fx,
                    'cy': fy,
                    'fx': fx,
                    'fy': fy,
                    'area': area,
                    'last_time': now,
                })
                unmatched_tracks.remove(best_id)
            else:
                self.tracks[self.next_track_id] = {
                    'cx': cx,
                    'cy': cy,
                    'fx': cx,
                    'fy': cy,
                    'area': area,
                    'last_time': now,
                }
                self.next_track_id += 1

        self._prune_tracks(now)
        return [(t.get('fx', t['cx']), t.get('fy', t['cy']), t['area']) for t in self.tracks.values()]

    # ------------------------------------------------
    def on_image(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        centers, mask = self.find_green_centers_and_mask(frame)
        now = self.get_clock().now()
        tracked_centers = self._update_tracks(centers, now)

        # ===== publish PoseArray (pixel coords) =====
        out = PoseArray()
        out.header = msg.header
        for cx, cy, area in tracked_centers:
            pose = Pose()
            pose.position.x = float(cx)  # u
            pose.position.y = float(cy)  # v
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            out.poses.append(pose)
        self.point_pub.publish(out)

        # ===== visualization =====
        if self.debug_view:
            vis = frame.copy()
            for cx, cy, area in tracked_centers:
                u, v = int(round(cx)), int(round(cy))
                cv2.circle(vis, (u, v), 6, (0, 0, 255), -1)
                cv2.putText(
                    vis, f"({u},{v}) a={area}",
                    (u + 8, v - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2
                )

            cv2.imshow("green_mask", mask)
            cv2.imshow("green_vis", vis)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = GreenBlobDetectorCompressed()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

