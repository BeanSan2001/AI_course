import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import subprocess
from launch.actions import (OpaqueFunction, LogInfo, RegisterEventHandler)
from launch.event_handlers import (OnProcessExit)


# Generate launch description
def generate_launch_description():

    config_arg = DeclareLaunchArgument(
        'config',
        default_value='params_gazebo.yaml',
        description='Name of the configuration file to load'
    )
    
    # Ghép đường dẫn bằng Substitution ➜ an toàn ở runtime
    config_path = PathJoinSubstitution([
        get_package_share_directory('controller_ex'),
        'config',
        LaunchConfiguration('config')
    ])


    # Node definition
    node = Node(
        package='controller_ex',
        executable='collision_detection.py',
        output='screen',
        emulate_tty=True,
        parameters=[config_path]
    )

    return LaunchDescription([
        config_arg,
        node
    ])
