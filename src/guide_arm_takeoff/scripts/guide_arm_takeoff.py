import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped
import time

class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')

        # Set up clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # Publisher for local position
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.set_mode_client.wait_for_service()
        self.arming_client.wait_for_service()
        self.takeoff_client.wait_for_service()
        self.get_logger().info('Services available. Starting mission...')

        # Delay to establish connections
        time.sleep(2)

        # Run steps
        self.set_mode('GUIDED')
        self.arm(True)
        self.takeoff(10.0)  # Altitude in meters

        self.timer = self.create_timer(10.0, self.goto_waypoint)

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().mode_sent:
            self.get_logger().info(f'Mode changed to {mode}')
        else:
            self.get_logger().error(f'Failed to set mode to {mode}')

    def arm(self, should_arm):
        req = CommandBool.Request()
        req.value = should_arm
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Vehicle armed')
        else:
            self.get_logger().error('Failed to arm vehicle')

    def takeoff(self, altitude):
        req = CommandTOL.Request()
        req.altitude = altitude
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Takeoff to {altitude}m successful')
        else:
            self.get_logger().error('Takeoff failed')
        time.sleep(10)

    def goto_waypoint(self):
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.pose.position.x = 10.0
        target.pose.position.y = 5.0
        target.pose.position.z = 10.0
        target.pose.orientation.w = 1.0
        self.get_logger().info('Sending waypoint...')
        for _ in range(100):
            self.local_pos_pub.publish(target)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
