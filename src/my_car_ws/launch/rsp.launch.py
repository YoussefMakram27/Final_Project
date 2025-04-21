from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package
    pkg_share = FindPackageShare("my_car_ws")

    # Path to URDF
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "my_car.urdf.xacro"])

    # Launch Configs
    urdf = LaunchConfiguration('urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare arguments
    declare_urdf = DeclareLaunchArgument(
        name='urdf',
        default_value=urdf_path,
        description='Path to URDF or XACRO file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )

    # Node: robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf])
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_urdf,
        declare_use_sim_time,
        robot_state_publisher_node
    ])
