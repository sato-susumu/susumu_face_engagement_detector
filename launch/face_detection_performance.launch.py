#!/usr/bin/env python3
"""
Face Detection Performance Test Launch
CPU負荷軽減機能のテスト用ランチファイル
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パッケージディレクトリ取得
    pkg_dir = get_package_share_directory('susumu_face_engagement_detector')
    
    # 設定ファイルパス
    config_file = os.path.join(pkg_dir, 'config', 'face_detection_performance.yaml')
    
    # 起動引数
    performance_mode_arg = DeclareLaunchArgument(
        'performance_mode',
        default_value='balanced',
        description='Performance mode: high, balanced, low, ultra_low'
    )
    
    # 性能モードに基づく設定
    performance_mode = LaunchConfiguration('performance_mode')
    
    # 静的変換パブリッシャー
    static_transform_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_link']
    )
    
    # 顔検出ノード（性能設定付き）
    face_detection_node = Node(
        package='susumu_face_engagement_detector',
        executable='face_detection_node',
        parameters=[config_file],
        output='screen'
    )
    
    # 顔認識ノード
    face_recognition_node = Node(
        package='susumu_face_engagement_detector',
        executable='face_recognition_node',
        parameters=[{
            'known_faces_dir': 'known_faces',
            'match_tolerance': 0.6
        }],
        output='screen'
    )
    
    # 注視分析ノード
    gaze_analysis_node = Node(
        package='susumu_face_engagement_detector',
        executable='gaze_analysis_node',
        parameters=[{
            'gaze_threshold_px': 50,
            'gaze_duration': 2.0
        }],
        output='screen'
    )
    
    # エンゲージメント管理ノード
    engagement_manager_node = Node(
        package='susumu_face_engagement_detector',
        executable='engagement_manager_node',
        parameters=[{
            'face_timeout': 1.0
        }],
        output='screen'
    )
    
    return LaunchDescription([
        performance_mode_arg,
        static_transform_pub,
        face_detection_node,
        face_recognition_node,
        gaze_analysis_node,
        engagement_manager_node
    ])