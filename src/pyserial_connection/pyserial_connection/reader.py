import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range, Temperature
from std_msgs.msg import String
import serial
import threading

class SensorReaderNode(Node):
    def __init__(self):
        super().__init__('sensor_reader_node')
        
        # Declare serial port parameter
        self.declare_parameter('serial_port', '/dev/pts/6')
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
            
        # Create publishers
        self.ultrasonic_left_pub = self.create_publisher(Range, '/ultrasonic/left', 10)
        self.ultrasonic_right_pub = self.create_publisher(Range, '/ultrasonic/right', 10)
        self.ultrasonic_front_pub = self.create_publisher(Range, '/ultrasonic/front', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.bme_pub = self.create_publisher(Temperature, '/bme/temperature', 10)
        self.air_quality_pub = self.create_publisher(String, '/air_quality', 10)
        
        # Start serial reading thread
        self.running = True
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.start()
        
        self.get_logger().info('Sensor reader node started')
    
    def read_serial(self):
        while self.running and rclpy.ok():
            try:
                # Read line from serial
                line = self.serial.readline().decode('utf-8').strip()
                if line:
                    # Parse data (expected format: ultrasonic_left,ultrasonic_right,ultrasonic_front,imux,imuy,imuz,bme,airquality)
                    data = line.split(',')
                    if len(data) == 8:
                        try:
                            # Extract values
                            ultrasonic_left = float(data[0])
                            ultrasonic_right = float(data[1])
                            ultrasonic_front = float(data[2])
                            imu_x = float(data[3])
                            imu_y = float(data[4])
                            imu_z = float(data[5])
                            bme_temp = float(data[6])
                            air_quality = data[7]
                            
                            # Get current time
                            current_time = self.get_clock().now().to_msg()
                            
                            # Publish ultrasonic data
                            range_left = Range()
                            range_left.header.stamp = current_time
                            range_left.header.frame_id = 'ultrasonic_left'
                            range_left.range = ultrasonic_left
                            range_left.radiation_type = Range.ULTRASOUND
                            range_left.field_of_view = 0.261799  # 15 degrees in radians
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
    
    def destroy_node(self):
        self.running = False
        if self.serial_thread.is_alive():
            self.serial_thread.join()
        if self.serial.is_open:
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SensorReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()