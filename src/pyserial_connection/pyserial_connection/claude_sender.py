#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import threading
import time

class STM32MotorNode(Node):
    def __init__(self):
        super().__init__('stm32_motor_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('debug_level', 'info')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.debug_level = self.get_parameter('debug_level').value
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f"Connected to STM32 on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to STM32: {e}")
            raise
        
        # Define QoS profile for teleop keyboard compatibility
        teleop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriber for velocity commands from teleop_keyboard
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile=teleop_qos
        )
        
        # Lock for serial write operations
        self.serial_lock = threading.Lock()
        
        # Record the last command and time to avoid spamming
        self.last_command = ""
        self.last_command_time = time.time()
        self.command_timeout = 0.1  # seconds
        
        # Available speeds for forward motion
        self.speeds = [300, 260, 120]  # Available speeds (max to min)
        self.current_speed_index = 0  # Track current speed
        
        self.get_logger().info("STM32 Motor node initialized")
        self.get_logger().info("Ready for teleop keyboard commands")

    def cmd_vel_callback(self, msg):
        """Process velocity commands from teleop_keyboard and send to STM32"""
        # Get current time
        now = time.time()
        
        # Convert twist message to motor commands
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Determine direction based on linear and angular velocities
        if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
            # Stop command
            command = 'S'
        elif linear_x > 0.0 and abs(angular_z) < 0.1:
            # Forward command with speed based on current speed index
            speed = self.speeds[self.current_speed_index]
            command = f"{speed:03d}"  # Format as 3 digits with leading zeros
            
            # Cycle to next speed for next forward command
            self.current_speed_index = (self.current_speed_index + 1) % len(self.speeds)
            self.get_logger().info(f"Forward speed: {speed}")
        elif linear_x < 0.0 and abs(angular_z) < 0.1:
            # Backward command
            command = 'B'
        elif angular_z > 0.0:
            # Left turn
            command = 'L'
        elif angular_z < 0.0:
            # Right turn
            command = 'R'
        else:
            # Default to stop
            command = 'S'
        
        # Only send if command changed or enough time has passed
        if command != self.last_command or (now - self.last_command_time) > self.command_timeout:
            self.send_command(command)
            self.last_command = command
            self.last_command_time = now

    def send_command(self, command):
        """Send command to STM32 with thread safety"""
        try:
            with self.serial_lock:
                self.ser.write(command.encode('utf-8'))
                self.ser.flush()
                
                if self.debug_level == 'debug':
                    self.get_logger().debug(f"Sent command: {command}")
                else:
                    self.get_logger().info(f"Sent command: {command}")
                
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

    def cleanup(self):
        """Clean up before node shutdown"""
        if hasattr(self, 'ser') and self.ser.is_open:
            # Stop motors before disconnecting
            try:
                with self.serial_lock:
                    self.ser.write(b'S')
                    self.ser.flush()
                    time.sleep(0.1)
                    self.ser.close()
                    self.get_logger().info("Closed serial connection")
            except Exception as e:
                self.get_logger().error(f"Error closing serial connection: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = STM32MotorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()