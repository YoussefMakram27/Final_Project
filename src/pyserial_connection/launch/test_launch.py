from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pyserial_connection',
            executable='readerandsender.py',
            name='stm_communicator_node',
            parameters=[
                {'serial_port': '/dev/pts/6'},
                {'baud_rate': 115200}
            ]
        ),
        Node(
            package='pyserial_connection',
            executable='yolo_simulator.py',
            name='yolo_simulator_node'
        ),
        Node(
            package='pyserial_connection',
            executable='sensorfusion.py',
            name='sensor_fusion_node'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                 '0.1', '0', '0.1', '0', '0', '0', 'base_link', 'camera_frame'],
            output='screen'
        )
    ])