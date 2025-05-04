import rclpy
from rclpy.node import Node
import serial
import struct
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Initialize serial communication
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.send_motor_command,
            10
        )

        self.get_logger().info("MotorController Node Started")

    def send_motor_command(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Differential drive formula to calculate the speed of the motor
        motor_speed = linear_speed + angular_speed

        # Convert motor speed to PWM range (e.g., -255 to 255)
        pwm_value = int(motor_speed * 255)

        # Clamp value to int16 range (you can adjust this if needed)
        pwm_value = max(-32768, min(32767, pwm_value))

        try:
            self.serial_port.reset_input_buffer()

            # Pack single PWM value into binary (little-endian signed 16-bit integer)
            data = struct.pack('<h', pwm_value)  # <h for little-endian int16

            # Send the binary data
            self.serial_port.write(data)
            self.serial_port.flush()

            self.get_logger().info(f"Sent PWM value: {pwm_value}")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

def main():
    rclpy.init()
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
