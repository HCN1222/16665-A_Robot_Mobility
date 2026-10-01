from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='motion_planning', executable='rrt_node.py', output='screen'),
    ])
