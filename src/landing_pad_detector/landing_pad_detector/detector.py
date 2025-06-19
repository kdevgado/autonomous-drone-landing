import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import time

class CombinedDetector(Node):
    def __init__(self):
        super().__init__('combined_detector')
        self.publisher_ = self.create_publisher(String, 'landing_status', 10)
        self.cap = cv2.VideoCapture(4)  # Change camera index as needed
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            exit()
        self.timer = self.create_timer(0.05, self.timer_callback)  # ~20 FPS

    def is_centered(self, bbox, frame_shape, tolerance=30):
        x, y, w, h = bbox
        cx = x + w // 2
        cy = y + h // 2
        frame_cx = frame_shape[1] // 2
        frame_cy = frame_shape[0] // 2
        return abs(cx - frame_cx) < tolerance and abs(cy - frame_cy) < tolerance

    def timer_callback(self):
        start = time.time()
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Camera frame not received.")
            return

        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        status_msg = String()
        detected = False

        # Color detection
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        moments = cv2.moments(mask)

        if moments["m00"] > 10000:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            detected = True
            if self.is_centered((cx-15, cy-15, 30, 30), frame.shape):
                status_msg.data = "LANDING_SAFE_COLOR"
                self.get_logger().info("detected landing pad")
            else:
                status_msg.data = "ADJUSTING_COLOR"
        else:
            # Fallback edge detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if 1000 < area < 10000:
                    approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                    if len(approx) >= 4:
                        x, y, w, h = cv2.boundingRect(approx)
                        cx = x + w // 2
                        cy = y + h // 2
                        detected = True
                        if self.is_centered((x, y, w, h), frame.shape):
                            status_msg.data = "LANDING_SAFE_EDGE"
                        else:
                            status_msg.data = "ADJUSTING_EDGE"
                        break

        if not detected:
            status_msg.data = "SEARCHING_MARKER"

        self.publisher_.publish(status_msg)
        end = time.time()
        elapsed_ms = (end - start) * 1000
        self.get_logger().info(f"Detection Time: {elapsed_ms:.2f} ms, Status: {status_msg.data}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CombinedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

