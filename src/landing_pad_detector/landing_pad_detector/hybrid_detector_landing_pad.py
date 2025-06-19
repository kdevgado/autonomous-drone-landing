
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class CombinedDetector(Node):
    def __init__(self):
        super().__init__('combined_detector')
        self.publisher_ = self.create_publisher(String, 'landing_status', 10)
        self.image_publisher = self.create_publisher(Image, 'annotated_landing_feed', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(4)  # Change camera index as needed
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            exit()
        self.timer = self.create_timer(0.05, self.timer_callback)

    def is_centered(self, cx, cy, frame_shape, tolerance=30):
        frame_cx = frame_shape[1] // 2
        frame_cy = frame_shape[0] // 2
        return abs(cx - frame_cx) < tolerance and abs(cy - frame_cy) < tolerance

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Camera frame not received.")
            return

        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        status_msg = String()

        # Detect blue circle
        lower_blue = np.array([100, 120, 60])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        landing_detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                circularity = area / (np.pi * radius * radius)
                if 0.8 < circularity < 1.2:
                    cx, cy, r = int(x), int(y), int(radius)
                    cv2.circle(frame, (cx, cy), r, (255, 0, 0), 2)

                    # Check for yellow cross inside
                    yellow_mask = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
                    combined_mask = cv2.bitwise_and(blue_mask, yellow_mask)
                    cross_contours, _ = cv2.findContours(combined_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                    for cross_cnt in cross_contours:
                        if cv2.contourArea(cross_cnt) > 500:
                            x2, y2, w2, h2 = cv2.boundingRect(cross_cnt)
                            cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 0), 2)

                            if self.is_centered(cx, cy, frame.shape):
                                status_msg.data = "LANDING_SAFE_HYBRID"
                            else:
                                status_msg.data = "ADJUSTING_HYBRID"

                            landing_detected = True
                            break
                    break

        if not landing_detected:
            status_msg.data = "SEARCHING_HYBRID"

        self.publisher_.publish(status_msg)
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_publisher.publish(image_msg)
        self.get_logger().info(f"Status: {status_msg.data}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CombinedDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
