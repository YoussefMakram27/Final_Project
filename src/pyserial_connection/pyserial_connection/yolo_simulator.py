import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseWithCovariance
import random

class YoloSimulatorNode(Node):
    def __init__(self):
        super().__init__('yolo_simulator_node')
        self.publisher = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.timer = self.create_timer(0.1, self.publish_detections)
        self.get_logger().info('YOLO simulator node started')

    def publish_detections(self):
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        num_detections = random.randint(0, 3)
        for _ in range(num_detections):
            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.x = random.uniform(0.0, 1.0)  # Correct Pose2D field
            detection.bbox.center.y = random.uniform(0.0, 1.0)  # Correct Pose2D field
            detection.bbox.size_x = random.uniform(0.1, 0.3)
            detection.bbox.size_y = random.uniform(0.1, 0.3)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = random.choice(['person', 'object', 'obstacle'])
            hypothesis.hypothesis.score = random.uniform(0.5, 1.0)
            hypothesis.pose.pose.position.z = random.uniform(0.5, 5.0)
            detection.results.append(hypothesis)

            msg.detections.append(detection)

        self.publisher.publish(msg)
        self.get_logger().info(f"Published {len(msg.detections)} YOLO detections")

def main(args=None):
    rclpy.init(args=args)
    node = YoloSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()