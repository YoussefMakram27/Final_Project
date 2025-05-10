import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class SerialReader(Node):
    def __init__(self):  # ✅ صح: double underscore
        super().__init__(node_name='serial_reader')
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.topic_name = 'serial_data'
        self.publisher_ = self.create_publisher(String, self.topic_name, 10)

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Opened serial port {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        self.timer = self.create_timer(0.1, self.read_serial_and_publish)

    def read_serial_and_publish(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            msg = String()
            msg.data = line
            self.get_logger().info(f"Publishing: {line}")
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  # ✅ صح
    main()
