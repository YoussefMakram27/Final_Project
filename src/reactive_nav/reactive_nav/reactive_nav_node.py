import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class ReactiveNavNode(Node):
    def __init__(self):
        super().__init__('reactive_nav_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.emergency_stop = False
        self.rotating = False
        self.rotation_start_time = None
        self.min_rotation_angle = math.radians(30)  # Minimum 30 degrees rotation
        self.angular_speed = 1.0  # rad/s for rotation
        self.rotation_direction = 0.0
        self.initial_move_logged = False  # Flag to log initial forward movement once

    def scan_callback(self, msg: LaserScan):
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
            # Decide rotation direction based on clearer side
            self.rotation_direction = -1.0 if min_right > min_left else 1.0
            self.get_logger().info(f'Robot stopped completely. Decided to rotate {"RIGHT" if self.rotation_direction < 0 else "LEFT"}')

        elif self.emergency_stop and not self.rotating:
            # Start rotating in place after complete stop
            twist.linear.x = 0.0
            twist.angular.z = self.rotation_direction * self.angular_speed
            self.rotating = True
            self.rotation_start_time = time.time()
            self.get_logger().info(f'Starting rotation in place to scan for clear space...')

        elif self.emergency_stop and self.rotating:
            # Continue rotating until minimum angle is reached
            elapsed_time = time.time() - self.rotation_start_time
            rotated_angle = abs(self.angular_speed * elapsed_time)

            if rotated_angle < self.min_rotation_angle:
                twist.linear.x = 0.0
                twist.angular.z = self.rotation_direction * self.angular_speed
                self.get_logger().info(f'Rotating: {math.degrees(rotated_angle):.1f}/{math.degrees(self.min_rotation_angle):.1f} degrees')
            else:
                # Check if path is clear after minimum rotation
                if min_front > 0.5:
                    self.get_logger().info('Clear path found, resuming forward motion.')
                    self.emergency_stop = False
                    self.rotating = False
                    self.rotation_start_time = None
                    twist.linear.x = 0.3
                    twist.angular.z = 0.0
                else:
                    # Path still blocked, continue rotating in place
                    self.get_logger().info('Path still blocked, continuing rotation in place...')
                    twist.linear.x = 0.0
                    twist.angular.z = self.rotation_direction * self.angular_speed
                    # Reset rotation start time to track next 30-degree increment
                    self.rotation_start_time = time.time()

        else:
            # Normal navigation logic
            if min_front > 2.0:
                if not self.initial_move_logged:
                    self.get_logger().info('Car is moving forward: Starting navigation.')
                    self.initial_move_logged = True
                twist.linear.x = 0.3
                twist.angular.z = 0.0
            elif 1.2 < min_front <= 2.0:
                self.get_logger().info(f'Obstacle ahead at {min_front:.2f}m: Slowing down.')
                twist.linear.x = 0.15
                twist.angular.z = 0.0
            else:
                self.get_logger().info(f'Very close obstacle at {min_front:.2f}m: Deciding turn...')
                if min_left > min_right and min_left > 0.5:
                    self.get_logger().info('Turning LEFT (left side clearer)')
                    twist.linear.x = 0.0
                    twist.angular.z = 1.5
                elif min_right > 0.5:
                    self.get_logger().info('Turning RIGHT (right side clearer)')
                    twist.linear.x = 0.0
                    twist.angular.z = -1.5
                else:
                    self.get_logger().info('Both sides blocked! Turning LEFT by default')
                    twist.linear.x = 0.0
                    twist.angular.z = 1.5

        self.cmd_vel_pub.publish(twist)

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
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C detected, stopping robot...')
        node.stop_robot()  # Publish stop command before shutdown
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()