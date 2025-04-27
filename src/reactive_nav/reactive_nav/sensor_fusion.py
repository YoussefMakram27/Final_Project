import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.range_sub = self.create_subscription(
            Range,
            '/range',
            self.range_callback,
            10
        )
        
        # Publisher
        self.fused_scan_pub = self.create_publisher(LaserScan, '/fused_scan', 10)

        # Initialize variables
        self.lidar_data = None
        self.range_data = None

    def scan_callback(self, msg: LaserScan):
        self.lidar_data = msg
        self.fuse_data()

    def range_callback(self, msg: Range):
        self.range_data = msg
        self.fuse_data()

    def fuse_data(self):
        if self.lidar_data is None or self.range_data is None:
            return
        
        # Combine the data - Example: Use range data for center or add as another scan point
        fused_scan = LaserScan()
        
        # Copy all the fields from the original scan
        fused_scan.header = Header()
        fused_scan.header.stamp = self.get_clock().now().to_msg()
        fused_scan.header.frame_id = self.lidar_data.header.frame_id
        fused_scan.angle_min = self.lidar_data.angle_min
        fused_scan.angle_max = self.lidar_data.angle_max
        fused_scan.angle_increment = self.lidar_data.angle_increment
        fused_scan.time_increment = self.lidar_data.time_increment
        fused_scan.scan_time = self.lidar_data.scan_time
        fused_scan.range_min = self.lidar_data.range_min
        fused_scan.range_max = self.lidar_data.range_max

        # Here, you can choose how to combine them, for instance, adding the range data to the middle of the scan
        fused_scan.ranges = self.lidar_data.ranges
        # Add the range data at a specific index in the scan or replace it
        middle_index = len(fused_scan.ranges) // 2
        fused_scan.ranges[middle_index] = self.range_data.range  # Example fusion logic

        # Publish the fused message
        self.fused_scan_pub.publish(fused_scan)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
