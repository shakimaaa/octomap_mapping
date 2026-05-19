from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('octomap_mapping'),
        'config',
        'fast_lio.yaml'
    )

    return LaunchDescription([
        Node(
            package='octomap_mapping',
            executable='octomap_mapping_node',
            name='octomap_mapping_node',
            output='screen',
            parameters=[config_file],
        )
    ])
