"""Start the RRT planner (open RViz yourself, e.g. with the simulator).

  ros2 launch motion_planning rrt_launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='motion_planning', executable='rrt_node.py', output='screen'),
    ])
