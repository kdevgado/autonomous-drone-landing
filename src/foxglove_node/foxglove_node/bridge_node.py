import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from foxglove_node_msgs.msg import StateInfo, BatteryInfo, EscInfo, EscArrayInfo
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from mavros_msgs.msg import ESCTelemetry
from mavros_msgs.msg import State

class FoxgloveBridge(Node):
    def __init__(self):
        super().__init__('foxglove_bridge')

        self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/mavros/battery', self.battery_cb, qos_profile=qos_profile_sensor_data)
#        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.velocity_cb, qos_profile=qos_profile_sensor_data)
        self.create_subscription(ESCTelemetry, '/mavros/esc_telemetry/telemetry', self.esc_telemetry_cb, qos_profile = qos_profile_sensor_data)

        self.state_pub = self.create_publisher(StateInfo, 'foxglove_bridge/state', 10)
        self.battery_pub = self.create_publisher(BatteryInfo, '/foxglove_bridge/battery',10)
        self.esc_telemetry_pub = self.create_publisher(EscArrayInfo, 'foxglove_bridge/esc_telemetry',10)

    def state_cb(self, msg):
        formatted = (
        f"Connected: {msg.connected}, Armed: {msg.armed}, Guided: {msg.guided}, "
        f"Manual Input: {msg.manual_input}, Mode: {msg.mode}, System Status: {msg.system_status}"
    )
        self.get_logger().info(formatted)

        msg_out = StateInfo()
        msg_out.connected = msg.connected
        msg_out.armed = msg.armed
        msg_out.guided = msg.guided
        msg_out.manual_input = msg.manual_input
        msg_out.mode = msg.mode
        msg_out.system_status = msg.system_status
        self.state_pub.publish(msg_out)

    def esc_telemetry_cb(self, msg):
        formatted = ""
        msg_out = EscArrayInfo()

        rpms = []
        currents = []
        temperatures = []

        for esc in msg.esc_telemetry:
            formatted += f"ESC: RPM={esc.rpm}, Current={esc.current}A, Temp={esc.temperature}°C\n"
            self.get_logger().info(formatted)

            rpms.append(float(esc.rpm))
            currents.append(round(float(esc.current),1))
            temperatures.append(float(esc.temperature))

            msg_out.rpms = rpms
            msg_out.currents = currents
            msg_out.temperatures = temperatures

#            esc_msg = EscInfo()
#            esc_msg.rpm = float(esc.rpm)
#            esc_msg.current = float(esc.current)
#            esc_msg.temperature = float(esc.temperature)
            self.esc_telemetry_pub.publish(msg_out)

    def imu_cb(self, msg):
        pass  # Add processing or publishing if needed

    def battery_cb(self, msg):
        voltage = msg.voltage
        current = msg.current
        temperature = msg.temperature
        present = msg.present
        formatted = f"Battery - Voltage: {voltage:.2f} V, Current: {current:.2f} A, Temp: {temperature:.2f} °C, Present: {present}"
        self.get_logger().info(formatted)
        msg_out = BatteryInfo()
        msg_out.voltage = msg.voltage
        msg_out.current = msg.current
        msg_out.temperature = msg.temperature
        msg_out.present = msg.present
        self.battery_pub.publish(msg_out)

    def velocity_cb(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

