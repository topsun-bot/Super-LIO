import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'rsairy_relocation.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'relocation.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')

    # ---- rslidar driver node ----
    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        output='screen',
    )

    # ---- Super-LIO relocalization node ----
    relocation_node = Node(
        package='super_lio',
        executable='relocation_node',
        name='relocation_node',
        output='screen',
        parameters=[config_yaml],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # ---- rviz ----
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        condition=IfCondition(rviz_flag),
    )

    # Delay relocalization to let rslidar driver start first
    delayed_reloc = TimerAction(
        period=3.0,
        actions=[relocation_node],
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_arg)
    # ld.add_action(rslidar_node)
    ld.add_action(delayed_reloc)
    ld.add_action(rviz2_node)

    return ld
