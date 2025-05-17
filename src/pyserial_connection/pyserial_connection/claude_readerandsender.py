#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu, Temperature
from std_msgs.msg import String, Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import threading
import time
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class STM32Interface(Node):
    def __init__(self):
        super().__init__('stm32_interface')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('debug_level', 'info')  # Add debug level parameter
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
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
        
        # Create publishers for sensor data
        self.ultrasonic_left_pub = self.create_publisher(Range, 'ultrasonic/left', 10)
        self.ultrasonic_right_pub = self.create_publisher(Range, 'ultrasonic/right', 10)
        self.ultrasonic_front_pub = self.create_publisher(Range, 'ultrasonic/front', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.temp_pub = self.create_publisher(Temperature, 'sensors/temperature', 10)
        self.gas_pub = self.create_publisher(String, 'sensors/air_quality', 10)
        
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
        
        # Initialize transform broadcaster for IMU
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create a timer for publishing data
        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)
        
        # Buffer for serial data
        self.serial_buffer = ""
        
        # Lock for serial read/write operations
        self.serial_lock = threading.Lock()
        
        # Flag to track if we're currently sending a command to avoid read/write conflicts
        self.sending_command = False
        
        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        # Record the last command and time to avoid spamming
        self.last_command = ""
        self.last_command_time = time.time()
        self.command_timeout = 0.1  # seconds
        
        # Available speeds for forward motion
        self.speeds = [300, 260, 120]  # Available speeds (max to min)
        self.current_speed_index = 0  # Track current speed
        
        # For tracking data reception
        self.last_data_time = time.time()
        self.data_count = 0
        
        self.get_logger().info("STM32 Interface node initialized")
        self.get_logger().info("Ready for teleop keyboard commands")

    def read_serial_data(self):
        """Thread function to continuously read serial data"""
        while rclpy.ok():
            try:
                # Don't read if we're sending a command to avoid conflicts
                if self.sending_command:
                    time.sleep(0.01)  # Short sleep to avoid busy waiting
                    continue
                
                with self.serial_lock:
                    if self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting).decode('utf-8')
                        # Log raw data if in debug mode
                        if self.debug_level == 'debug':
                            self.get_logger().debug(f"Read raw data: {data.strip()}")
                        self.serial_buffer += data
                        
                        # Process complete lines
                        while '\n' in self.serial_buffer:
                            line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                            if line.strip():  # Skip empty lines
                                # Log each complete line received
                                self.get_logger().info(f"Received data: {line.strip()}")
                                self.process_sensor_data(line.strip())
                    
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(1.0)  # Wait a bit before retrying

    def process_sensor_data(self, data):
        """Process sensor data received from STM32"""
        try:
            # Parse comma-separated values
            # Expected format: ultrasonic_left,ultrasonic_right,ultrasonic_front,imux,imuy,imuz,temperature,air_quality
            values = data.split(',')
            
            if len(values) != 8:
                self.get_logger().warn(f"Received malformed data: {data} (expected 8 values, got {len(values)})")
                return
            
            # Extract values (with error checking)
            try:
                ultrasonic_left = int(values[0])
                ultrasonic_right = int(values[1])
                ultrasonic_front = int(values[2])
                imu_x = int(values[3])
                imu_y = int(values[4])
                imu_z = int(values[5])
                temperature = float(values[6])
                air_quality = values[7]
                
                # Track data reception statistics
                self.data_count += 1
                now = time.time()
                if now - self.last_data_time >= 5.0:  # Log every 5 seconds
                    rate = self.data_count / (now - self.last_data_time)
                    self.get_logger().info(f"Data reception rate: {rate:.2f} Hz (received {self.data_count} packets in {now - self.last_data_time:.2f}s)")
                    self.data_count = 0
                    self.last_data_time = now
                
                # Log processed values if in debug level
                if self.debug_level == 'debug':
                    self.get_logger().debug(f"Processed values: ultrasonic=[{ultrasonic_left}, {ultrasonic_right}, {ultrasonic_front}], " + 
                                           f"imu=[{imu_x}, {imu_y}, {imu_z}], temp={temperature}, gas={air_quality}")
                
                # Publish all sensor data
                self.publish_ultrasonic_data(ultrasonic_left, ultrasonic_right, ultrasonic_front)
                self.publish_imu_data(imu_x, imu_y, imu_z)
                self.publish_temperature(temperature)
                self.publish_air_quality(air_quality)
                
            except (ValueError, IndexError) as e:
                self.get_logger().warn(f"Error parsing sensor data: {e}, raw data: {data}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing sensor data: {e}")

    def publish_ultrasonic_data(self, left, right, front):
        """Publish ultrasonic sensor data"""
        # Helper function to create and fill Range message
        def create_range_msg(distance, frame_id):
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26  # ~15 degrees in radians
            msg.min_range = 0.02      # 2cm
            msg.max_range = 4.0       # 400cm
            msg.range = distance / 100.0  # Convert cm to meters
            return msg
        
        # Publish data for each ultrasonic sensor
        self.ultrasonic_left_pub.publish(create_range_msg(left, 'ultrasonic_left_link'))
        self.ultrasonic_right_pub.publish(create_range_msg(right, 'ultrasonic_right_link'))
        self.ultrasonic_front_pub.publish(create_range_msg(front, 'ultrasonic_front_link'))

    def publish_imu_data(self, x, y, z):
        """Publish IMU data"""
        # Create and fill IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Convert raw accelerometer values to m/s^2
        # Note: These conversion factors depend on your IMU settings
        # For MPU6050 with FS_ACC_4G setting:
        accel_scale = 9.81 * 4.0 / 32768.0  # 4g range, convert to m/s^2
        
        # Set linear accelerations
        imu_msg.linear_acceleration.x = x * accel_scale
        imu_msg.linear_acceleration.y = y * accel_scale
        imu_msg.linear_acceleration.z = z * accel_scale
        
        # Set covariance (example values, should be tuned)
        accel_cov = 0.01  # m/s^2
        imu_msg.linear_acceleration_covariance = [
            accel_cov, 0.0, 0.0,
            0.0, accel_cov, 0.0,
            0.0, 0.0, accel_cov
        ]
        
        # Orientation is not provided by the STM32 code, set to identity quaternion
        imu_msg.orientation.w = 1.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        
        # Set orientation covariance to -1 indicating orientation not available
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Angular velocity not provided
        imu_msg.angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Publish the IMU message
        self.imu_pub.publish(imu_msg)
        
        # Broadcast IMU transform
        self.broadcast_imu_tf()

    def broadcast_imu_tf(self):
        """Broadcast IMU transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        
        # Set translation (example position)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        
        # Set rotation (identity)
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)

    def publish_temperature(self, temp):
        """Publish temperature data"""
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'temperature_sensor'
        msg.temperature = temp
        msg.variance = 0.05  # Example variance value
        
        self.temp_pub.publish(msg)

    def publish_air_quality(self, quality):
        """Publish air quality data"""
        msg = String()
        if quality == 'G':
            msg.data = "Gas detected"
        else:
            msg.data = "Normal"
        
        self.gas_pub.publish(msg)

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
            # Set flag to indicate sending command
            self.sending_command = True
            
            with self.serial_lock:
                self.ser.write(command.encode('utf-8'))
                self.ser.flush()
                self.get_logger().info(f"Sent command: {command}")
                
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
        finally:
            # Reset flag
            self.sending_command = False

    def timer_callback(self):
        """Timer callback for periodic tasks"""
        # Check if we're receiving data (add a warning if no data for a while)
        now = time.time()
        if now - self.last_data_time > 5.0 and self.data_count == 0:
            self.get_logger().warn("No sensor data received in the last 5 seconds")

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
    node = STM32Interface()
    
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