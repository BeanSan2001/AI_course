import os, launch_ros
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('amfitrack'),
        'config',
        'params.yaml'
        )

    node = launch_ros.actions.Node(
        package='amfitrack',
        executable='amfitrack.py',
        output='screen',
        emulate_tty=True,
        parameters=[config])

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


    return LaunchDescription([
        left_static_tf,
        right_static_tf,
        node
    ])
