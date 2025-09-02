from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    coordinator_pkg_prefix = get_package_share_directory('coordinator')

    # param.yaml 경로
    coordinator_params_file = os.path.join(
        coordinator_pkg_prefix, 'config', 'params.yaml')

    declare_params = DeclareLaunchArgument(
        'coordinator_params_file',
        default_value=coordinator_params_file,
        description='Path to config file for coordinator'
    )

    coordinator_node = Node(
        package='coordinator',
        executable='coordinator_node',
        name='coordinator',
        output='screen',
        parameters=[
            LaunchConfiguration('coordinator_params_file')
        ]
    )

    return LaunchDescription([
        declare_params,
        coordinator_node
    ])

