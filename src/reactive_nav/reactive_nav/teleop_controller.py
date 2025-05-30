import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Node Started.....")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial: {e}")
            raise

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.send_motor_command,
            10
        )

    def send_motor_command(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        try:
            if linear > 0.01:
                speed = min(999, int(linear * 1000))  # Cap at 999
                command = f"{speed:03d}"  # Zero-pad to 3 digits (e.g., "050")

            elif linear < -0.01:  # Moving backward
                command = 'B'
                self.get_logger().info("Sending BACKWARD command")

            elif angular > 0.5:  # Turning left
                command = 'L'
                self.get_logger().info("Turning LEFT")

            elif angular < -0.5:  # Turning right
                command = 'R'
                self.get_logger().info("Turning RIGHT")

            else:
                command = 'S'  # Stop
                self.get_logger().info("Stopping robot")

            self.serial_port.reset_input_buffer()
            self.serial_port.write(command.encode())
            self.serial_port.flush()
            self.get_logger().info(f"Sent to serial: {command}")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

def main():
    rclpy.init()
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()