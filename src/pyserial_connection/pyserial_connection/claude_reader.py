#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Imu, Range, Temperature
from geometry_msgs.msg import Vector3

class SensorDataReader(Node):
    def __init__(self):
        super().__init__('sensor_data_reader')
        
        # Serial port settings
        self.declare_parameter('port', '/dev/pts/6')
        self.declare_parameter('baudrate', 115200)
        
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # Create publishers for each sensor using standard ROS message types
        # Ultrasonic sensors using Range message type
        self.ultrasonic_left_pub = self.create_publisher(Range, 'ultrasonic/left', 10)
        self.ultrasonic_right_pub = self.create_publisher(Range, 'ultrasonic/right', 10)
        self.ultrasonic_front_pub = self.create_publisher(Range, 'ultrasonic/front', 10)
        
        # IMU using Imu message type
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # BME using Temperature message type
        self.bme_pub = self.create_publisher(Temperature, 'bme/temperature', 10)
        
        # Air quality (keeping as String since it's not standard)
        self.air_quality_pub = self.create_publisher(String, 'air_quality', 10)
        
        # Debug publisher
        self.debug_pub = self.create_publisher(String, 'sensor_debug', 10)
        
        # Initialize serial port
        self.get_logger().info(f"Opening {self.port} at {self.baudrate} baud")
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.ser.reset_input_buffer()
            self.get_logger().info("Serial port opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port: {e}")
            rclpy.shutdown()
            return
            
        # Buffer for collecting bytes until a complete message
        self.buffer = bytearray()
        
        # Create timer to read from serial port
        self.timer = self.create_timer(0.05, self.read_serial)
        
        # Statistics
        self.messages_received = 0
        self.parsing_errors = 0
        self.stats_timer = self.create_timer(5.0, self.print_stats)
    
    def read_serial(self):
        """Read from serial port and process any complete messages"""
        if not self.ser.is_open:
            return
            
        try:
            # Read any available data
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                
                # Add to our buffer
                self.buffer.extend(data)
                
                # Process any complete messages in buffer
                self.process_buffer()
                
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            if self.ser.is_open:
                self.ser.close()
    
    def process_buffer(self):
        """Process the buffer for complete messages ending with newline"""
        while b'\n' in self.buffer:
            # Find the end of the first complete message
            newline_pos = self.buffer.find(b'\n')
            
            # Extract the message and convert to string
            message = self.buffer[:newline_pos].decode('utf-8', errors='replace').strip()
            
            # Remove the processed message from buffer
            self.buffer = self.buffer[newline_pos + 1:]
            
            # Parse and publish the data
            self.parse_and_publish(message)
    
    def parse_and_publish(self, message):
        """Parse the message and publish to appropriate topics"""
        # Expected format: ultrasonic_left,ultrasonic_right,ultrasonic_front,imu_accel_x,imu_accel_y,imu_accel_z,bme,airquality
        self.messages_received += 1
        
        # Debug publish
        debug_msg = String()
        debug_msg.data = f"Raw message: {message}"
        self.debug_pub.publish(debug_msg)
        
        try:
            # Split the message by commas
            parts = message.split(',')
            
            # Ensure we have the expected number of values
            if len(parts) != 8:
                self.get_logger().warning(f"Received message with {len(parts)} values, expected 8: {message}")
                self.parsing_errors += 1
                return
            
            # Parse ultrasonic values
            ultrasonic_left = float(parts[0])
            ultrasonic_right = float(parts[1])
            ultrasonic_front = float(parts[2])
            
            # Parse IMU linear acceleration values (not orientation)
            imu_accel_x = float(parts[3])
            imu_accel_y = float(parts[4])
            imu_accel_z = float(parts[5])
            
            # Parse BME value
            bme_value = float(parts[6])
            
            # Parse air quality (as a letter)
            air_quality = parts[7]
            
            # Publish ultrasonic data with Range message type
            self.publish_range(self.ultrasonic_left_pub, ultrasonic_left, "ultrasonic/left")
            self.publish_range(self.ultrasonic_right_pub, ultrasonic_right, "ultrasonic/right")
            self.publish_range(self.ultrasonic_front_pub, ultrasonic_front, "ultrasonic/front")
            
            # Publish IMU data with Imu message type
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            # Set linear acceleration values
            imu_msg.linear_acceleration.x = imu_accel_x
            imu_msg.linear_acceleration.y = imu_accel_y
            imu_msg.linear_acceleration.z = imu_accel_z
            
            # Set other fields to identity/zero
            imu_msg.orientation.w = 1.0
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            
            # Set covariance to unknown
            imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            imu_msg.angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]  # Known acceleration
            
            self.imu_pub.publish(imu_msg)
            
            # Publish BME data with Temperature message type
            temp_msg = Temperature()
            temp_msg.header.stamp = self.get_clock().now().to_msg()
            temp_msg.header.frame_id = "bme_link"
            temp_msg.temperature = bme_value
            temp_msg.variance = 0.0  # Set to a suitable value if known
            self.bme_pub.publish(temp_msg)
            
            # Publish air quality data
            air_quality_msg = String()
            air_quality_msg.data = air_quality
            self.air_quality_pub.publish(air_quality_msg)
            
            # Consolidated logging of all sensor values
            self.get_logger().info(
                f"Ultrasonic: L={ultrasonic_left}, R={ultrasonic_right}, F={ultrasonic_front}, "
                f"IMU accel: x={imu_accel_x}, y={imu_accel_y}, z={imu_accel_z}, "
                f"BME Temp: {bme_value}, Air Quality: {air_quality}"
            )
            
        except ValueError as e:
            self.get_logger().error(f"Error parsing message: {e}")
            self.parsing_errors += 1
    
    def publish_range(self, publisher, value, topic_name):
        """Helper to publish a Range message with proper fields"""
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = topic_name.split('/')[-1] + "_link"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.1  # Typical value in radians
        msg.min_range = 0.02  # Typical minimum range in meters
        msg.max_range = 4.0   # Typical maximum range in meters
        msg.range = value
        publisher.publish(msg)
    
    def print_stats(self):
        """Print statistics"""
        self.get_logger().info("\n--- SENSOR STATISTICS ---")
        self.get_logger().info(f"Messages received: {self.messages_received}")
        self.get_logger().info(f"Parsing errors: {self.parsing_errors}")
        self.get_logger().info(f"Success rate: {(self.messages_received - self.parsing_errors) / max(1, self.messages_received):.2%}")
        self.get_logger().info("------------------------\n")

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("Serial port closed")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()