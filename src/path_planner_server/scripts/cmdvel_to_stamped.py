#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        self.get_logger().info("cmd_vel_bridge node started: /cmd_vel ➝ /mavros/setpoint_velocity/cmd_vel (TwistStamped)")

    def cmd_vel_callback(self, msg):
        # Convert Twist to TwistStamped
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = "base_link"
        stamped_msg.twist = msg

        self.publisher.publish(stamped_msg)
        self.get_logger().debug(f"Published TwistStamped: {stamped_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
