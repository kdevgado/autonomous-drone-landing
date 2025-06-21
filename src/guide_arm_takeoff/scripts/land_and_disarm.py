import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandTOL, CommandBool
import time

class LandAndDisarmNode(Node):

    def __init__(self):
        super().__init__('land_and_disarm_node')

        # Service clients
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.get_logger().info('Waiting for services...')
        self.land_client.wait_for_service()
        self.arming_client.wait_for_service()
        self.get_logger().info('Services available.')

        # Start landing sequence
        self.land()
        time.sleep(10)  # Give time to land
        self.disarm()

    def land(self):
        req = CommandTOL.Request()
        req.altitude = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0

        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Landing initiated successfully')
        else:
            self.get_logger().error('Landing failed')

    def disarm(self):
        req = CommandBool.Request()
        req.value = False  # False = disarm

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Vehicle disarmed successfully')
        else:
            self.get_logger().error('Failed to disarm vehicle')


def main(args=None):
    rclpy.init(args=args)
    node = LandAndDisarmNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

