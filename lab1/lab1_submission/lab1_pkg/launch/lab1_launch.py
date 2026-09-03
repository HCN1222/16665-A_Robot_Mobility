from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('lab1_pkg'),
        'config',
        'talker_params.yaml'
    )

    talker_node = Node(
        package='lab1_pkg',
        executable='talker.py',
        name='talker',
        parameters=[config]
    )

    relay_node = Node(
        package='lab1_pkg',
        executable='relay.py',
        name='relay',
    )

    ld.add_action(talker_node)
    ld.add_action(relay_node)

    return ld
