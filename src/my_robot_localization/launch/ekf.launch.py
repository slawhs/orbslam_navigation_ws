from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    localization_pkg_share = FindPackageShare(package='my_robot_localization').find('my_robot_localization')
    ekf_config_path = os.path.join(localization_pkg_share, 'config/ekf.yaml') 

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True}],
        )
    ])
