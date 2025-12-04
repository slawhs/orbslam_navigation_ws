from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    voc = LaunchConfiguration('voc', default='ORBvoc.txt')
    settings = LaunchConfiguration('settings', default='orb_slam3/config/RGB-D/RealSense_D435i.yaml')
    topic_rgb = LaunchConfiguration('topic_rgb', default='/camera/color/image_raw')
    topic_depth = LaunchConfiguration('topic_depth', default='/camera/aligned_depth_to_color/image_raw')

    return LaunchDescription([
        DeclareLaunchArgument('voc', default_value=voc),
        DeclareLaunchArgument('settings', default_value=settings),
        DeclareLaunchArgument('topic_rgb', default_value=topic_rgb),
        DeclareLaunchArgument('topic_depth', default_value=topic_depth),

        Node(
            package='ros2_orb_slam3',
            executable='rgbd_example',
            name='orb_slam3_rgbd',
            output='screen',
            parameters=[{
                'voc_path': voc,
                'settings_path': settings,
                'topic_rgb': topic_rgb,
                'topic_depth': topic_depth
            }]
        )
    ])