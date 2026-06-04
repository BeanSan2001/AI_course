import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_urdf_path = get_package_share_directory('scout_simulation')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )
    
    model_arg = DeclareLaunchArgument(
        'model', default_value='scout_mini_real.urdf.xacro',
        description='Name of the URDF description to load'
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        pkg_urdf_path,  
        "urdf", "robots",
        LaunchConfiguration('model')  
    ])

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_urdf_path, 'rviz', 'scout_real.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': False},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': False},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # body 和 base_scan 之間的靜態 TF 轉換 (平移和旋轉皆為 0.0)
    static_tf_body_base_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_base_scan',
        arguments=[
            '0', '0', '0',   # x y z
            '0', '0', '0',   # roll pitch yaw
            'body',          # parent frame
            'base_scan'      # child frame
        ]
    )
    
    # === 外部 Launch 檔案引入 ===
    
    laser_dir = get_package_share_directory('pointcloud_to_laserscan')
    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(laser_dir, 'launch', 'pointcloud_to_laserscan_launch.py'))
    )
    
    cost_map_dir = get_package_share_directory('cost_map')
    cost_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cost_map_dir, 'launch', 'cost_map.launch.py'))
    )

    point_dir = get_package_share_directory('optimal_point')
    point_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(point_dir, 'launch', 'optimal_point.launch.py'))
    )
   
    # scout_base
    scout_base_dir = get_package_share_directory('scout_base')
    scout_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(scout_base_dir, 'launch', 'scout_mini_omni_base.launch.py')),
        launch_arguments={'publish_tf': 'false'}.items()
    )

    # Livox MID360 
    livox_dir = get_package_share_directory('livox_ros_driver2')
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(livox_dir, 'launch', 'msg_MID360_launch.py'))
    )

    #LaunchDescription
    launchDescriptionObject = LaunchDescription()
    
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(static_tf_body_base_scan)
    
    # launchDescriptionObject.add_action(laser_launch)
    # launchDescriptionObject.add_action(cost_map_launch)
    # launchDescriptionObject.add_action(point_launch)
    launchDescriptionObject.add_action(scout_base_launch)
    # launchDescriptionObject.add_action(livox_launch)

    return launchDescriptionObject