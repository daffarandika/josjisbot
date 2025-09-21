from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    is_robot_centric = LaunchConfiguration("is_robot_centric")

    robot_centric_controller = Node(
        package="josjisbot_controller",
        executable="robot_centric_controller",
        output="screen",
        condition=IfCondition(is_robot_centric),
    )

    world_centric_controller = Node(
        package="josjisbot_controller",
        executable="world_centric_controller",
        output="screen",
        condition=UnlessCondition(is_robot_centric),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "is_robot_centric",
            default_value="false",
        ),
        robot_centric_controller,
        world_centric_controller,
    ])
