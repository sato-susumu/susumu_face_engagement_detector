from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # パッケージパスを取得
    pkg_share = FindPackageShare('susumu_face_engagement_detector')
    
    # RViz設定ファイルのパス
    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'face_detection_rviz.rviz'
    ])
    
    # 起動引数定義
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Static transform for camera frame
    static_transform_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_link'],
        output='screen'
    )
    
    # RViz2ノード
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        static_transform_pub,
        rviz_node
    ])