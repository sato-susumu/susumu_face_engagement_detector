from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Static transform for camera frame
    static_transform_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_link'],
        output='screen'
    )
    
    # 顔検出ノード
    face_detection_node = Node(
        package='susumu_face_engagement_detector',
        executable='face_detection_node',
        name='face_detection_node',
        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'detection_model': 'hog'
        }],
        output='screen'
    )
    
    # 顔認識ノード
    face_recognition_node = Node(
        package='susumu_face_engagement_detector',
        executable='face_recognition_node',
        name='face_recognition_node',
        parameters=[{
            'known_faces_dir': 'known_faces',
            'match_tolerance': 0.6
        }],
        output='screen'
    )
    
    # RViz2ノード（設定ファイルなし）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        static_transform_pub,
        face_detection_node,
        face_recognition_node,
        rviz_node
    ])