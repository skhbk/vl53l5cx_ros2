from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg


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

    vl53l5cx_node = LifecycleNode(
        package="vl53l5cx",
        executable="ranging",
        name="vl53l5cx",
        namespace="",
        parameters=[parameters],
        emulate_tty=True,
    )

    configure_node = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(vl53l5cx_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_node = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=vl53l5cx_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(vl53l5cx_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(LaunchConfiguration("activate_on_configure")),
    )

    actions = [
        vl53l5cx_node,
        configure_node,
        activate_node,
    ]

    return actions


def generate_launch_description():
    args = []

    args.append(DeclareLaunchArgument("config_package", default_value="vl53l5cx"))
    args.append(DeclareLaunchArgument("config_file", default_value=""))
    args.append(DeclareLaunchArgument("activate_on_configure", default_value="false"))

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
