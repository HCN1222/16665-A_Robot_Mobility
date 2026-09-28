"""Start the RRT planner together with an RViz window showing the grid,
tree, path and goal.

  ros2 launch motion_planning rrt_launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('motion_planning'),
                               'rviz', 'rrt_sim.rviz')
    return LaunchDescription([
        Node(package='motion_planning', executable='rrt_node.py', output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config]),
    ])
