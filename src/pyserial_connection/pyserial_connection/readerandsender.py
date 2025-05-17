import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range, Temperature
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import threading
import time

class STMCommunicatorNode(Node):
    def __init__(self):
        super().__init__('stm_communicator_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Initialize serial connection
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return
            
        # Serial lock for thread safety
        self.serial_lock = threading.Lock()
        
        # Create publishers
        self.ultrasonic_left_pub = self.create_publisher(Range, '/ultrasonic/left', 10)
        self.ultrasonic_right_pub = self.create_publisher(Range, '/ultrasonic/right', 10)
        self.ultrasonic_front_pub = self.create_publisher(Range, '/ultrasonic/front', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.bme_pub = self.create_publisher(Temperature, '/bme/temperature', 10)
        self.air_quality_pub = self.create_publisher(String, '/air_quality', 10)
        
        # Create subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Motor speed settings
        self.speeds = [300, 260, 120]  # Available speeds
        self.current_speed_index = 0  # Track current speed
        
        # Start serial reading thread
        self.running = True
        self.serial_thread = threading.Thread(target=self.read_sensor_data)
        self.serial_thread.start()
        
        self.get_logger().info('STM communicator node started')
    
    def cmd_vel_callback(self, msg):
        """Handle incoming cmd_vel messages and send motor commands."""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Determine motor command
        if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
            command = "S"  # Stop
        elif linear_x > 0:
            # Forward: cycle through speeds
            speed = self.speeds[self.current_speed_index]
            command = f"{speed}"
            self.current_speed_index = (self.current_speed_index + 1) % len(self.speeds)
        elif linear_x < 0:
            command = "B"  # Backward
        elif angular_z > 0:
            command = "L"  # Left
        elif angular_z < 0:
            command = "R"  # Right
        
        # Send command to STM
        with self.serial_lock:
            try:
                self.serial.reset_output_buffer()
                self.serial.write(f"{command}".encode('utf-8'))
                self.serial.flush()
                self.get_logger().info(f"Sent motor command: {command}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send motor command: {e}")
    
    def read_sensor_data(self):
        while self.running and rclpy.ok():
            with self.serial_lock:
                try:
                    # Clear any old data in the buffer
                    self.serial.reset_input_buffer()
                    
                    # Read line from serial
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().info(f"Raw received: {line}")  # Debug raw data
                        
                        # Parse data
                        data = line.split(',')
                        if len(data) == 8:
                            try:
                                # Extract values - add more error checking
                                ultrasonic_left = float(data[0])
                                ultrasonic_right = float(data[1])
                                ultrasonic_front = float(data[2])
                                imu_x = float(data[3])
                                imu_y = float(data[4])
                                imu_z = float(data[5])
                                bme_temp = float(data[6])
                                air_quality = data[7].strip()  # Ensure no extra whitespace

                                # Log received data
                                self.get_logger().info(
                                    f"Ultrasonic: L={ultrasonic_left}, R={ultrasonic_right}, F={ultrasonic_front}, "
                                    f"IMU: x={imu_x}, y={imu_y}, z={imu_z}, "
                                    f"BME Temp: {bme_temp}, Air Quality: {air_quality}"
                                )
                                
                                # Get current time
                                current_time = self.get_clock().now().to_msg()
                                
                                # Publish ultrasonic data
                                range_left = Range()
                                range_left.header.stamp = current_time
                                range_left.header.frame_id = 'ultrasonic_left'
                                range_left.range = ultrasonic_left
                                range_left.radiation_type = Range.ULTRASOUND
                                range_left.field_of_view = 0.261799
                                range_left.min_range = 0.02
                                range_left.max_range = 4.0
                                self.ultrasonic_left_pub.publish(range_left)
                                
                                range_right = Range()
                                range_right.header.stamp = current_time
                                range_right.header.frame_id = 'ultrasonic_right'
                                range_right.range = ultrasonic_right
                                range_right.radiation_type = Range.ULTRASOUND
                                range_right.field_of_view = 0.261799
                                range_right.min_range = 0.02
                                range_right.max_range = 4.0
                                self.ultrasonic_right_pub.publish(range_right)
                                
                                range_front = Range()
                                range_front.header.stamp = current_time
                                range_front.header.frame_id = 'ultrasonic_front'
                                range_front.range = ultrasonic_front
                                range_front.radiation_type = Range.ULTRASOUND
                                range_front.field_of_view = 0.261799
                                range_front.min_range = 0.02
                                range_front.max_range = 4.0
                                self.ultrasonic_front_pub.publish(range_front)
                                
                                # Publish IMU data
                                imu_msg = Imu()
                                imu_msg.header.stamp = current_time
                                imu_msg.header.frame_id = 'imu'
                                imu_msg.linear_acceleration.x = imu_x
                                imu_msg.linear_acceleration.y = imu_y
                                imu_msg.linear_acceleration.z = imu_z
                                self.imu_pub.publish(imu_msg)
                                
                                # Publish BME temperature
                                temp_msg = Temperature()
                                temp_msg.header.stamp = current_time
                                temp_msg.header.frame_id = 'bme'
                                temp_msg.temperature = bme_temp
                                self.bme_pub.publish(temp_msg)
                                
                                # Publish air quality
                                air_quality_msg = String()
                                air_quality_msg.data = air_quality
                                self.air_quality_pub.publish(air_quality_msg)
                                
                            except ValueError as e:
                                self.get_logger().warn(f"Invalid data format: {e}")
                        else:
                            self.get_logger().warn(f"Unexpected data length: {len(data)}")
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial error: {e}")
                    break
                except UnicodeDecodeError:
                    self.get_logger().warn("Failed to decode serial data")
            time.sleep(0.01)  # Avoid busy-waiting
    
    def destroy_node(self):
        """Clean up resources."""
        self.running = False
        if self.serial_thread.is_alive():
            self.serial_thread.join()
        with self.serial_lock:
            if self.serial.is_open:
                self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STMCommunicatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()