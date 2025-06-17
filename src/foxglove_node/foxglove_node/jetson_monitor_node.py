[200~import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json

try:
    from jtop import jtop
except ImportError:
    jtop = None

class JetsonMonitor(Node):
    def __init__(self):
        super().__init__('jetson_monitor')
        self.publisher_ = self.create_publisher(String, 'jetson_stats', 1)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Jetson monitor node started')

    def timer_callback(self):
        stats = {
            cpu_usage: psutil.cpu_percent(interval=None),
            ram_usage_percent: psutil.virtual_memory().percent,
        }

        if jtop:
            with jtop() as jetson:
                if jetson.ok():
                    jetson_data = jetson.stats
                    stats.update({
                        power: jetson_data.get(power, {}),
                        temp: jetson_data.get(temp, {}),
                        gpu: jetson_data.get(gpu, {}),
                    })

        msg = String()
        msg.data = json.dumps(stats)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Jetson stats: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = JetsonMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
~
