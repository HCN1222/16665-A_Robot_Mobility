"""Start Pure Pursuit (open RViz yourself, e.g. with the simulator).

  ros2 launch pure_pursuit pure_pursuit_launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='pure_pursuit', executable='pure_pursuit_node.py', output='screen'),
    ])
