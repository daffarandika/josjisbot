import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():
    pkg_name = 'josjisbot_description'

    urdf_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'josjisbot.urdf.xacro'
    )

    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Node RViz2
    rviz_config_dir = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'display.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
