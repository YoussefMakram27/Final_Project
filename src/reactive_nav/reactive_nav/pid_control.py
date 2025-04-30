import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time
import signal
import sys


class ReactiveNavNode(Node):
    def __init__(self):
        super().__init__('reactive_nav_node')

        self.register_signal_handlers()
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Subscribe to teleop twist commands
        self.teleop_sub = self.create_subscription(
            Twist, 
            '/teleop_cmd_vel',  # Topic from teleop_twist_keyboard
            self.teleop_callback, 
            10
        )

        # Timer to check if teleop is active
        self.create_timer(0.5, self.check_teleop_timeout)
        
        # Reactive navigation variables
        self.emergency_stop = False
        self.rotating = False
        self.rotation_start_time = None
        self.min_rotation_angle = math.radians(30)  # Minimum 30 degrees rotation
        self.angular_speed = 1.0  # rad/s for rotation
        self.rotation_direction = 0.0
        self.initial_move_logged = False  # Flag to log initial forward movement once

        # PID variables for rotation control
        self.kp = 1.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.5  # Derivative gain
        self.previous_error = 0.0
        self.integral = 0.0

        # Direction control
        self.turning_direction = None  # 'left' or 'right'
        
        # Teleop control variables
        self.manual_control = False
        self.last_teleop_time = time.time()
        self.teleop_timeout = 0.5  # Return to reactive after this many seconds without teleop input
        
        self.get_logger().info('Reactive navigation with teleop override initialized')
        self.get_logger().info('Run teleop_twist_keyboard in another terminal to take manual control')
        self.get_logger().info('Command: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros_args -r /cmd_vel:=/teleop_cmd_vel')

    def pid_control(self, target_angle, current_angle):
        """
        Calculate the control signal using PID for smooth rotation.
        """
        error = target_angle - current_angle
        self.integral += error
        derivative = error - self.previous_error

        # Compute PID output
        pid_output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Update previous error for the next calculation
        self.previous_error = error

        return pid_output
    
    def teleop_callback(self, msg):
        """Handle incoming teleop commands"""
        # Mark teleop as active
        self.manual_control = True
        self.last_teleop_time = time.time()
        
        # If this is the first teleop message after reactive mode
        if not self.manual_control:
            self.get_logger().info('Teleop control activated')
            
        # Forward the teleop command to the robot
        self.cmd_vel_pub.publish(msg)
    
    def check_teleop_timeout(self):
        """Check if teleop has timed out and we should return to reactive mode"""
        if self.manual_control and (time.time() - self.last_teleop_time > self.teleop_timeout):
            self.manual_control = False
            self.get_logger().info('Teleop timeout. Returning to reactive navigation.')

    def scan_callback(self, msg: LaserScan):
        # Skip reactive navigation logic if in manual control mode
        if self.manual_control:
            return
            
        ranges = list(msg.ranges)
        ranges = [r if 0.05 < r < 10.0 else 10.0 for r in ranges]

        n = len(ranges)
        front = ranges[n//2 - 45 : n//2 + 45]
        left = ranges[n//2 + 30 : n//2 + 60]
        right = ranges[n//2 - 60 : n//2 - 30]

        min_front = min(front)
        min_left = min(left)
        min_right = min(right)

        twist = Twist()

        if min_front < 0.5 and not self.emergency_stop:
            self.get_logger().warn(f'EMERGENCY BRAKE! Obstacle detected at {min_front:.2f}m!')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.emergency_stop = True
            self.rotating = False
            self.rotation_start_time = None
            self.turning_direction = None
            self.rotation_direction = -1.0 if min_right > min_left else 1.0
            self.get_logger().info(f'Robot stopped completely. Decided to rotate {"RIGHT" if self.rotation_direction < 0 else "LEFT"}')

        elif self.emergency_stop and not self.rotating:
            twist.linear.x = 0.0
            twist.angular.z = self.rotation_direction * self.angular_speed
            self.rotating = True
            self.rotation_start_time = time.time()
            self.get_logger().info(f'Starting rotation in place to scan for clear space...')

        elif self.emergency_stop and self.rotating:
            elapsed_time = time.time() - self.rotation_start_time
            rotated_angle = abs(self.angular_speed * elapsed_time)

            if rotated_angle < self.min_rotation_angle:
                pid_output = self.pid_control(self.min_rotation_angle, rotated_angle)
                twist.linear.x = 0.0
                twist.angular.z = self.rotation_direction * pid_output
                self.get_logger().info(f'Rotating: {math.degrees(rotated_angle):.1f}/{math.degrees(self.min_rotation_angle):.1f} degrees')
            else:
                if min_front > 0.5:
                    self.get_logger().info('Clear path found, resuming forward motion.')
                    self.emergency_stop = False
                    self.rotating = False
                    self.rotation_start_time = None
                    twist.linear.x = 0.5
                    twist.angular.z = 0.0
                else:
                    self.get_logger().info('Path still blocked, continuing rotation in place...')
                    twist.linear.x = 0.0
                    pid_output = self.pid_control(self.min_rotation_angle, rotated_angle)
                    twist.angular.z = self.rotation_direction * pid_output
                    self.rotation_start_time = time.time()

        else:
            # Normal navigation logic
            if min_front > 2.0:
                if not self.initial_move_logged:
                    self.get_logger().info('Car is moving forward: Starting navigation.')
                    self.initial_move_logged = True
                twist.linear.x = 0.5
                twist.angular.z = 0.0
                self.turning_direction = None  # reset any previous turning decision
            elif 1.2 < min_front <= 2.0:
                # Gradually reduce speed by 0.03 for every 0.1m closer than 2.0m
                distance_to_obstacle = 2.0 - min_front
                decay_steps = int(distance_to_obstacle / 0.1)
                speed = max(0.03, 0.3 - (decay_steps * 0.03))  # Don't go below 0.03

                self.get_logger().info(f'Obstacle ahead at {min_front:.2f}m: Slowing down to {speed:.2f} m/s')
                twist.linear.x = speed
                twist.angular.z = 0.0
                self.turning_direction = None
            elif 0.5 < min_front <= 1.2:
                self.get_logger().info(f'Very close obstacle at {min_front:.2f}m: Deciding turn...')

                if self.turning_direction is None:
                    if min_left > min_right and min_left > 0.5:
                        self.turning_direction = 'left'
                        self.get_logger().info('Decided: Turning LEFT (left side clearer)')
                    elif min_right > 0.5:
                        self.turning_direction = 'right'
                        self.get_logger().info('Decided: Turning RIGHT (right side clearer)')
                    else:
                        self.turning_direction = 'left'
                        self.get_logger().info('Both sides blocked! Defaulting to turn LEFT')

                twist.linear.x = 0.0
                if self.turning_direction == 'left':
                    twist.angular.z = 1.5
                else:
                    twist.angular.z = -1.5
            else:
                self.get_logger().warn(f'Obstacle too close at {min_front:.2f}m! Stopping.')
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)


    def register_signal_handlers(self):
        signal.signal(signal.SIGINT, self.shutdown_handler)
        signal.signal(signal.SIGTERM, self.shutdown_handler)

    def shutdown_handler(self, signum, frame):
        self.get_logger().info('Shutdown signal received. Stopping robot...')
        self.stop_robot()
        rclpy.shutdown()
        sys.exit(0)


    def stop_robot(self):
        """Publish a zero-velocity command to stop the robot."""
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_twist)
        self.get_logger().info('Robot stopped: Zero velocity command published.')

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavNode()

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()