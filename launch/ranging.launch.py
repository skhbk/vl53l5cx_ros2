from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameters = ([
        get_package_share_directory('vl53l5cx'),
        '/config/',
        LaunchConfiguration('config_file', default='default.yaml')
    ])

    return LaunchDescription([
        Node(
            package='vl53l5cx',
            executable='ranging',
            parameters=[parameters]
        )
    ])
