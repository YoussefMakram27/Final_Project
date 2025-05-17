import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Initialize serial communication
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.send_motor_command, 10
        )
        
        self.get_logger().info("MotorController Node Started")

    def send_motor_command(self, msg):
        linear_speed = msg.linear.x  # Forward/Backward movement
        angular_speed = msg.angular.z  # Turning speed

        # Define motor speeds
        left_speed = linear_speed - angular_speed
        right_speed = linear_speed + angular_speed

        # Scale to a suitable range (e.g., -255 to 255 for PWM)
        left_pwm = int(left_speed * 255)
        right_pwm = int(right_speed * 255)

        # Ensure values are within limits
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        # Determine command based on movement
        if linear_speed > 0:  # Moving forward
            command = "F"
        elif linear_speed < 0:  # Moving backward
            command = "B"
        elif angular_speed > 0:  # Turning left in place
            command = "L"
        elif angular_speed < 0:  # Turning right in place
            command = "R"
        else:  # Stop
            command = "S"

        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(command.encode())
            self.serial_port.flush()

            self.get_logger().info(f"Sent command: {command}")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

        self.get_logger().info(f"{command.strip()}")

def main():
    rclpy.init()
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()