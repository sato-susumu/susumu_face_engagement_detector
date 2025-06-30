from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('susumu_face_engagement_detector')
    config_file = os.path.join(pkg_dir, 'config', 'face_detection_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to config file'
        ),
        
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='Input image topic name'
        ),
        
        Node(
            package='susumu_face_engagement_detector',
            executable='face_detection_node',
            name='face_detection_node',
            parameters=[
                LaunchConfiguration('config_file'),
                {'image_topic': LaunchConfiguration('image_topic')}
            ],
            output='screen'
        )
    ])