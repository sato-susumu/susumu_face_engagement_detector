#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージディレクトリ取得
    pkg_dir = get_package_share_directory('susumu_face_engagement_detector')
    
    # 起動引数定義
    refresh_rate_arg = DeclareLaunchArgument(
        'refresh_rate',
        default_value='1.0',
        description='Monitor refresh rate in seconds'
    )
    
    show_content_arg = DeclareLaunchArgument(
        'show_content',
        default_value='true',
        description='Show message content in display'
    )
    
    log_to_file_arg = DeclareLaunchArgument(
        'log_to_file',
        default_value='false',
        description='Enable logging to file'
    )
    
    # モニタリングノード
    monitoring_node = Node(
        package='susumu_face_engagement_detector',
        executable='monitoring_node',
        name='face_engagement_monitor',
        output='screen',
        parameters=[{
            'refresh_rate': LaunchConfiguration('refresh_rate'),
            'show_content': LaunchConfiguration('show_content'),
            'log_to_file': LaunchConfiguration('log_to_file'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        refresh_rate_arg,
        show_content_arg, 
        log_to_file_arg,
        monitoring_node,
    ])