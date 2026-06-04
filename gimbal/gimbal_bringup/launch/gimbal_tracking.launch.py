import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gimbal_bringup'),
        'config',
        'params.yaml'
        )
    
    left_static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_designed_point',
        arguments=[
            '0.7', '0.9', '0.0',          # x y z
            '0', '0', '0',                # roll pitch yaw (rad)
            'human_link', 'left_following'
        ],
        output='screen'
    )

    right_static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_designed_point',
        arguments=[
            '0.7', '-0.9', '0.0',          # x y z
            '0', '0', '0',                # roll pitch yaw (rad)
            'human_link', 'right_following'
        ],
        output='screen'
    )

    node = launch_ros.actions.Node(
        package='gimbal_bringup',
        executable='gimbal_tracking.py',
        output='screen',
        emulate_tty=True,
        parameters=[config])
    
    return LaunchDescription([
        left_static_tf,
        right_static_tf,
        node
    ])
