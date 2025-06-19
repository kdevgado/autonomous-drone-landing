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
        self.timer = self.create_timer(0.05, self.timer_callback)  # ~20 FPS


    def timer_callback(self):
        start = time.time()
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Camera frame not received.")
            return

        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Color detection
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        moments = cv2.moments(mask)

        if moments["m00"] > 10000:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            base_center = (cx, cy)

            # Draw center
            cv2.circle(frame, (cx, cy), 4, (255, 0, 0), -1)
            cv2.putText(frame, "Base", (cx - 30, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        else:
            # Edge-based fallback detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if 1000 < area < 10000:
                    approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                    if len(approx) >= 4 and cv2.isContourConvex(approx):
                        x, y, w, h = cv2.boundingRect(approx)
                        aspect_ratio = w / float(h)
                        if 0.85 < aspect_ratio < 1.15:
                            cx = x + w // 2
                            cy = y + h // 2
                            base_center = (cx, cy)

                            # Draw square and center
                            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                            cv2.circle(frame, (cx, cy), 4, (255, 0, 0), -1)
                            cv2.putText(frame, "Base", (x, y - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_publisher.publish(image_msg)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Actual FPS: {actual_fps}")

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