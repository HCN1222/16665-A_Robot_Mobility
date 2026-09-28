"""Start Pure Pursuit together with an RViz window showing the waypoints.

  ros2 launch pure_pursuit pure_pursuit_launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('pure_pursuit'),
                               'rviz', 'pure_pursuit_sim.rviz')
    return LaunchDescription([
        Node(package='pure_pursuit', executable='pure_pursuit_node.py', output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config]),
    ])
