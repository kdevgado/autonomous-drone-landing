import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped  # <-- For optical flow velocity output
from sensor_msgs.msg import Range
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class CombinedDetector(Node):
    def __init__(self):
        super().__init__('combined_detector')
        self.altitude = None
        self.altitude_sub = self.create_subscription(
            Range,
            '/mavros/rangefinder/rangefinder',  # Update this!
            self.altitude_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'landing_status', 10)
        self.image_publisher = self.create_publisher(Image, 'annotated_landing_feed', 10)
        self.velocity_publisher = self.create_publisher(TwistStamped, '/mavros/vision_velocity/sensor', 10)

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(4)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            exit()
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.prev_gray = None
        self.prev_pts = None

        # Add these two:
        self.focal_length_px = 700.0  # Replace with your camera focal length in pixels
        self.frame_rate = 20.0        # Your callback timer rate (20 Hz)

    def altitude_callback(self, msg):
        self.altitude = msg.range  # Get altitude in meters

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Camera frame not received.")
            return

        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Yellow color mask
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Morphological cleanup
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cross_found = False

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 150:
                continue

            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            if len(approx) >= 8:  # A cross may appear complex when viewed at angle
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = w / float(h)

                if 0.5 < aspect_ratio < 2.0:  # Rough filter: not too elongated
                    cx = x + w // 2
                    cy = y + h // 2
                    cv2.drawContours(frame, [approx], -1, (0, 255, 255), 2)
                    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                    cv2.putText(frame, "Cross", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cross_found = True
                    break  # Stop after first detection

        if not cross_found:
            cv2.putText(frame, "No cross detected", (10, 230),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # --- Optical Flow Computation (NEW) ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is not None and self.prev_pts is not None and len(self.prev_pts) > 0:
            # Calculate optical flow
            curr_pts, status, err = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray, self.prev_pts, None)

            # Filter good points
            good_prev_pts = self.prev_pts[status == 1]
            good_curr_pts = curr_pts[status == 1]

            # Compute average flow vector (in pixels)
            flow_vectors = good_curr_pts - good_prev_pts

            if len(flow_vectors) > 0:
                avg_flow = np.mean(flow_vectors, axis=0)  # (dx, dy) in pixels/frame

                if self.altitude is None:
                    self.get_logger().warning("No altitude data, cannot scale optical flow velocity.")
                    # Don't return here, continue to update prev_gray and prev_pts below

                else:
                    # Convert pixel displacement to meters using altitude and focal length
                    velocity_x = -(avg_flow[0] * self.altitude) / self.focal_length_px * self.frame_rate
                    velocity_y = -(avg_flow[1] * self.altitude) / self.focal_length_px * self.frame_rate

                    # Publish velocity message
                    twist_msg = TwistStamped()
                    twist_msg.header.stamp = self.get_clock().now().to_msg()
                    twist_msg.twist.linear.x = float(velocity_x)
                    twist_msg.twist.linear.y = float(velocity_y)
                    self.velocity_publisher.publish(twist_msg)

        # Update previous frame and points for next iteration (always update!)
        self.prev_gray = gray.copy()
        self.prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.01, minDistance=10)

        # -------------------------------------

        # Publish image
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_publisher.publish(image_msg)

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
