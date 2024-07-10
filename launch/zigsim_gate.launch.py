import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tf_tools'),
        'config',
        'param.yaml'
    )
    return LaunchDescription([
        Node(
            package='tf_tools',
            namespace='zigsim_gate',
            executable='zigsim_gate',
            remappings=[('/output_pose', '/iphone_pose')],
            parameters=[config]
        ),
    ])