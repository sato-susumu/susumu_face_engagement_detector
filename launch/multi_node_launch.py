from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 起動引数定義
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Input image topic name'
    )
    
    detection_model_arg = DeclareLaunchArgument(
        'detection_model',
        default_value='hog',
        description='Face detection model (hog or cnn)'
    )
    
    known_faces_dir_arg = DeclareLaunchArgument(
        'known_faces_dir',
        default_value='known_faces',
        description='Directory containing known face images'
    )
    
    match_tolerance_arg = DeclareLaunchArgument(
        'match_tolerance',
        default_value='0.6',
        description='Face matching tolerance threshold'
    )
    
    gaze_threshold_px_arg = DeclareLaunchArgument(
        'gaze_threshold_px',
        default_value='50',
        description='Gaze detection threshold in pixels'
    )
    
    gaze_duration_arg = DeclareLaunchArgument(
        'gaze_duration',
        default_value='2.0',
        description='Gaze duration threshold in seconds'
    )
    
    face_timeout_arg = DeclareLaunchArgument(
        'face_timeout',
        default_value='1.0',
        description='Face detection timeout in seconds'
    )

    # 1プロセス内で全ノードを実行するマルチノードエグゼキューター
    multi_node_executor = Node(
        package='susumu_face_engagement_detector',
        executable='multi_node_executor',
        name='multi_node_executor',
        output='screen'
    )

    return LaunchDescription([
        image_topic_arg,
        detection_model_arg,
        known_faces_dir_arg,
        match_tolerance_arg,
        gaze_threshold_px_arg,
        gaze_duration_arg,
        face_timeout_arg,
        multi_node_executor
    ])