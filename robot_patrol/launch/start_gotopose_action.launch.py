from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='pose_server_node',
            output='screen'),
    ])