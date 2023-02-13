from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    parameters = {}

    if LaunchConfiguration("config_file").perform(context):
        parameters = PathJoinSubstitution(
            [
                FindPackageShare(LaunchConfiguration("config_package")),
                "config",
                LaunchConfiguration("config_file"),
            ]
        )

    vl53l5cx_node = Node(
        package="vl53l5cx", executable="ranging_node", parameters=[parameters]
    )

    actions = [vl53l5cx_node]

    return actions


def generate_launch_description():
    args = []

    args.append(DeclareLaunchArgument("config_package", default_value="vl53l5cx"))
    args.append(DeclareLaunchArgument("config_file", default_value=""))

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
