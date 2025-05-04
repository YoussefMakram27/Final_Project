import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import serial
import math
import time
import signal
import sys

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # ************************
        # * Serial Communication *
        # ************************
        try:
            # Initialize serial connection to STM32
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Serial port connected to STM32")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise

        # ****************************
        # * ROS2 Publishers/Subscribers *
        # ****************************
        # Publisher for visualization (RViz, etc.)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for LIDAR data
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Subscriber for teleop commands
        self.teleop_sub = self.create_subscription(
            Twist, '/teleop_cmd_vel', self.teleop_callback, 10)

        # **********************
        # * Control Parameters *
        # **********************
        # Speed parameters (converted to 0-999 range for STM32)
        self.default_speed = 0.3  # m/s (300 when converted)
        self.min_speed = 0.03    # m/s (30 when converted)
        
        # Distance thresholds (meters)
        self.far_threshold = 1.0    # Start slowing down at 1.0m
        self.close_threshold = 0.2   # Start turning at 0.2m
        self.emergency_threshold = 0.05  # Emergency stop at 0.05m
        
        # Rotation parameters
        self.rotation_speed = 1.5  # rad/s
        self.min_rotation_angle = math.radians(30)  # Minimum 30 degree rotation
        
        # ********************
        # * State Variables *
        # ********************
        self.manual_control = False  # True when teleop is active
        self.last_teleop_time = time.time()
        self.teleop_timeout = 0.5  # Return to auto after 0.5s inactivity
        
        self.emergency_stop = False  # True when obstacle too close
        self.rotating = False        # True during obstacle avoidance rotation
        self.rotation_start_time = None
        self.rotation_direction = 0.0  # 1.0 for left, -1.0 for right
        self.turning_direction = None  # 'left' or 'right'
        
        # PID controller variables
        self.kp = 1.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.5  # Derivative gain
        self.previous_error = 0.0
        self.integral = 0.0

        # ********************
        # * Setup Timers/Signals *
        # ********************
        self.create_timer(0.5, self.check_teleop_timeout)  # Check teleop timeout
        self.register_signal_handlers()  # Handle Ctrl+C
        
        self.get_logger().info('Robot controller initialized')
        self.get_logger().info(f'Default speed: {self.default_speed} m/s')
        self.get_logger().info(f'Obstacle thresholds: Far={self.far_threshold}m, Close={self.close_threshold}m, Emergency={self.emergency_threshold}m')

    def send_motor_command(self, linear, angular):
        """
        Send motor commands to STM32 via serial.
        Converts ROS2 Twist messages to STM32 protocol:
        - Forward: "XXX" (3-digit speed 000-999)
        - Backward: 'B'
        - Left: 'L'
        - Right: 'R'
        - Stop: 'S'
        """
        try:
            if linear > 0.01:  # Forward motion
                # Convert m/s to 0-999 range (300 = default_speed)
                speed = min(999, max(0, int(linear * 1000)))
                command = f"{speed:03d}"  # Zero-pad to 3 digits ("050")
                self.get_logger().info(f"Forward at {speed}", throttle_duration_sec=1)
                
            elif linear < -0.01:  # Backward
                command = 'B'
                self.get_logger().warn("BACKWARD command")
                
            elif angular > 0.5:  # Left turn
                command = 'L'
                self.get_logger().info("LEFT turn")
                
            elif angular < -0.5:  # Right turn
                command = 'R'
                self.get_logger().info("RIGHT turn")
                
            else:  # Stop
                command = 'S'
                self.get_logger().info("STOPPING", throttle_duration_sec=1)
            
            # Send command to STM32
            self.serial_port.reset_input_buffer()
            self.serial_port.write(command.encode())
            self.serial_port.flush()
            
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication failed: {e}")

    def teleop_callback(self, msg):
        """Handle teleop keyboard commands"""
        self.manual_control = True
        self.last_teleop_time = time.time()
        
        # Forward commands directly to motors
        self.send_motor_command(msg.linear.x, msg.angular.z)
        
        # Also publish for visualization
        self.cmd_vel_pub.publish(msg)

    def check_teleop_timeout(self):
        """Return to autonomous mode if no teleop commands received"""
        if self.manual_control and (time.time() - self.last_teleop_time > self.teleop_timeout):
            self.manual_control = False
            self.get_logger().info('Returning to autonomous mode')

    def scan_callback(self, msg):
        """Autonomous navigation using LIDAR data"""
        if self.manual_control:
            return  # Skip autonomous control when in manual mode
            
        # Clean LIDAR data (replace invalid values with 10.0m)
        ranges = [r if 0.05 < r < 10.0 else 10.0 for r in msg.ranges]
        n = len(ranges)
        
        # Define detection zones:
        front = ranges[n//2 - 30 : n//2 + 30]  # 60° front cone
        left = ranges[n//2 : n//2 + 60]        # Right side (LIDAR is inverted)
        right = ranges[n//2 - 60 : n//2]       # Left side
        
        min_front = min(front)
        min_left = min(left)
        min_right = min(right)

        twist = Twist()

        # ******************************
        # * Emergency Stop Condition *
        # ******************************
        if min_front < self.emergency_threshold and not self.emergency_stop:
            self.emergency_stop = True
            self.rotating = False
            self.get_logger().warn(f'EMERGENCY BRAKE! Obstacle at {min_front:.2f}m!')
            
            # Decide rotation direction (whichever side has more space)
            self.rotation_direction = -1.0 if min_right > min_left else 1.0
            direction = "RIGHT" if self.rotation_direction < 0 else "LEFT"
            self.get_logger().warn(f'Will rotate {direction} to avoid obstacle')
            
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # **************************
        # * Obstacle Avoidance *
        # **************************
        elif self.emergency_stop:
            if not self.rotating:
                # Start rotating in place
                self.rotating = True
                self.rotation_start_time = time.time()
                twist.linear.x = 0.0
                twist.angular.z = self.rotation_direction * self.rotation_speed
            else:
                # Check if we've rotated enough
                elapsed = time.time() - self.rotation_start_time
                rotated_angle = abs(self.rotation_speed * elapsed)
                
                if rotated_angle < self.min_rotation_angle:
                    # Use PID for smooth rotation
                    pid_output = self.pid_control(self.min_rotation_angle, rotated_angle)
                    twist.angular.z = self.rotation_direction * pid_output
                else:
                    # After minimum rotation, check if path is clear
                    if min_front > self.close_threshold:
                        self.emergency_stop = False
                        self.rotating = False
                        twist.linear.x = self.default_speed
                        twist.angular.z = 0.0
                        self.get_logger().warn('Path clear, resuming navigation')
                    else:
                        # Continue rotating
                        self.rotation_start_time = time.time()
                        twist.angular.z = self.rotation_direction * self.rotation_speed

        # **************************
        # * Normal Navigation *
        # **************************
        else:
            if min_front > self.far_threshold:
                # Full speed ahead
                twist.linear.x = self.default_speed
                twist.angular.z = 0.0
                
            elif self.close_threshold < min_front <= self.far_threshold:
                # Calculate how many 0.1m steps we are within the danger zone
                distance_into_zone = self.far_threshold - min_front
                steps = int(distance_into_zone / 0.1)  # Number of 0.1m increments
                
                # Reduce speed by 10 units per step (converted back to m/s)
                speed_reduction = steps * 30
                current_speed = max(
                    self.min_speed, 
                    self.default_speed - (speed_reduction / 1000)  # Convert back to m/s
                )
                
                twist.linear.x = current_speed
                twist.angular.z = 0.0
                self.get_logger().info(
                    f'Reduced speed to {int(current_speed*1000):03d} '
                    f'(obstacle at {min_front:.2f}m, {steps} steps)',
                    throttle_duration_sec=1
                )
                
            elif min_front <= self.close_threshold:
                # Too close - need to turn
                twist.linear.x = 0.0
                
                if min_left > min_right and min_left > self.close_threshold:
                    twist.angular.z = self.rotation_speed  # Turn left
                    self.get_logger().warn('Turning LEFT to avoid obstacle')
                elif min_right > self.close_threshold:
                    twist.angular.z = -self.rotation_speed  # Turn right
                    self.get_logger().warn('Turning RIGHT to avoid obstacle')
                else:
                    twist.angular.z = self.rotation_speed  # Default left turn
                    self.get_logger().warn('Both sides blocked! Defaulting to LEFT turn')

        # **************************
        # * Execute Commands *
        # **************************
        self.send_motor_command(twist.linear.x, twist.angular.z)
        self.cmd_vel_pub.publish(twist)  # For visualization

    def pid_control(self, target, current):
        """PID controller for smooth rotation"""
        error = target - current
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def register_signal_handlers(self):
        """Handle shutdown signals gracefully"""
        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

    def shutdown(self, signum, frame):
        """Clean shutdown procedure"""
        self.get_logger().warn('SHUTDOWN SIGNAL RECEIVED')
        self.send_motor_command(0, 0)  # Stop motors
        self.serial_port.close()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    try:
        rclpy.spin(controller)
    except Exception as e:
        controller.get_logger().error(f'Fatal error: {e}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()