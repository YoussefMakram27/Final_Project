#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Temperature
from std_msgs.msg import String
import serial
import threading
import time

class STM32SensorNode(Node):
    def __init__(self):
        super().__init__('stm32_sensor_node')
        
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
        self.ultrasonic_leftback_pub = self.create_publisher(Range, 'ultrasonic/leftback', 10)
        self.ultrasonic_rightback_pub = self.create_publisher(Range, 'ultrasonic/rightback', 10)
        self.temp_pub = self.create_publisher(Temperature, 'sensors/temperature', 10)
        self.gas_pub = self.create_publisher(String, 'sensors/air_quality', 10)
        
        # Create a timer for publishing data
        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)
        
        # Buffer for serial data
        self.serial_buffer = ""
        
        # Lock for serial read/write operations
        self.serial_lock = threading.Lock()
        
        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        # For tracking data reception
        self.last_data_time = time.time()
        self.data_count = 0
        
        self.get_logger().info("STM32 Sensor node initialized")

    def read_serial_data(self):
        """Thread function to continuously read serial data"""
        while rclpy.ok():
            try:
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
                                if self.debug_level == 'debug':
                                    self.get_logger().debug(f"Received raw data: {line.strip()}")
                                self.process_sensor_data(line.strip())
                    
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(1.0)  # Wait a bit before retrying

    def process_sensor_data(self, data):
        """Process sensor data received from STM32"""
        try:
            # Parse comma-separated values
            # Expected format: ultrasonic_left,ultrasonic_right,ultrasonic_front,ultrasonic_leftback,ultrasonic_rightback,temperature,air_quality
            values = data.split(',')
            
            if len(values) != 7:
                self.get_logger().warn(f"Received malformed data: {data} (expected 7 values, got {len(values)})")
                return
            
            # Extract values (with error checking)
            try:
                ultrasonic_left = int(values[0])
                ultrasonic_right = int(values[1])
                ultrasonic_front = int(values[2])
                ultrasonic_leftback = int(values[3])
                ultrasonic_rightback = int(values[4])
                temperature = float(values[5])
                air_quality = values[6]
                
                # Track data reception statistics
                self.data_count += 1
                now = time.time()
                if now - self.last_data_time >= 5.0:  # Log every 5 seconds
                    rate = self.data_count / (now - self.last_data_time)
                    self.get_logger().info(f"Data reception rate: {rate:.2f} Hz (received {self.data_count} packets in {now - self.last_data_time:.2f}s)")
                    self.data_count = 0
                    self.last_data_time = now
                
                # Log processed values in terminal for all debug levels
                self.get_logger().info(
                    f"Ultrasonic: L={ultrasonic_left}, R={ultrasonic_right}, F={ultrasonic_front}, "
                    f"LB={ultrasonic_leftback}, RB={ultrasonic_rightback}, "
                    f"BME Temp: {temperature}, Air Quality: {air_quality}"
                )
                
                # Publish all sensor data
                self.publish_ultrasonic_data(ultrasonic_left, ultrasonic_right, ultrasonic_front, 
                                           ultrasonic_leftback, ultrasonic_rightback)
                self.publish_temperature(temperature)
                self.publish_air_quality(air_quality)
                
            except (ValueError, IndexError) as e:
                self.get_logger().warn(f"Error parsing sensor data: {e}, raw data: {data}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing sensor data: {e}")

    def publish_ultrasonic_data(self, left, right, front, leftback, rightback):
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
        self.ultrasonic_leftback_pub.publish(create_range_msg(leftback, 'ultrasonic_leftback_link'))
        self.ultrasonic_rightback_pub.publish(create_range_msg(rightback, 'ultrasonic_rightback_link'))

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

    def timer_callback(self):
        """Timer callback for periodic tasks"""
        # Check if we're receiving data (add a warning if no data for a while)
        now = time.time()
        if now - self.last_data_time > 5.0 and self.data_count == 0:
            self.get_logger().warn("No sensor data received in the last 5 seconds")

    def cleanup(self):
        """Clean up before node shutdown"""
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                with self.serial_lock:
                    self.ser.close()
                    self.get_logger().info("Closed serial connection")
            except Exception as e:
                self.get_logger().error(f"Error closing serial connection: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = STM32SensorNode()
    
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