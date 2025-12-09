from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/tomy/orbslam_navigation_ws/src/my_robot_navigation/maps/office_small.yaml'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_file
            }]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_footprint',
                'scan_topic': '/scan'
            }]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        )
    ])