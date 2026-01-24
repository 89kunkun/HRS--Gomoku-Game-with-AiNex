import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from servo_service.msg import FaceBoundingBox, FaceBoundingBoxArray

import numpy as np

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__("face_detection_node")

        self.bridge = CvBridge()

        # Mediapipe Face Detection (Solution API)
        self.mp_face = mp.solutions.face_detection
        self.face_detector = self.mp_face.FaceDetection(
            model_selection = 0,    # 0: short-range, 1: full-range
            min_detection_confidence = 0.5
        )
         
        # Subscribe to undistorted image
        self.create_subscription(
            Image,
            "/camera/image_undistorted",
            self.image_callback,
            10
        )

        # Publicher for bounding boxes
        self.pub_bbox = self.create_publisher(
            FaceBoundingBoxArray,
            "/mediapipe/face_bbox",
            10
        )
        
        self.get_logger().info("MediaPipe Face Detection (Solutions API) Started.")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Inference
        results = self.face_detector.process(rgb)

        # Bounding box arrray
        bbox_array = FaceBoundingBoxArray()
        bbox_array.header = msg.header

        if results.detections:
            for detection in results.detections:

                # Mediapipe provides bounding box in normalied form
                box = detection.location_data.relative_bounding_box

                x = box.xmin * frame.shape[1]
                y = box.ymin * frame.shape[0]
                w = box.width * frame.shape[1]
                h = box.height * frame.shape[0]

                # Create bounding box message
                bbox_msg = FaceBoundingBox()
                bbox_msg.xmin = float(x)
                bbox_msg.ymin = float(y)
                bbox_msg.width = float(w)
                bbox_msg.height = float(h)

                bbox_array.boxes.append(bbox_msg)

                # Draw bounding box on image
                cv2.rectangle(
                    frame,
                    (int(x), int(y)),
                    (int(x + w), int(y + h)),
                    (0, 255, 0), 2
                )

        # Publish bounding boxes
        self.pub_bbox.publish(bbox_array)

        # Display image
        cv2.imshow("Mediapipe Face Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()