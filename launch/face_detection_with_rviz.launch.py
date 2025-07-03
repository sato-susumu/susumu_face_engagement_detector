from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パッケージパスとRViz設定ファイル
    try:
        pkg_dir = get_package_share_directory('susumu_face_engagement_detector')
        rviz_config_file = os.path.join(pkg_dir, 'config', 'face_detection_rviz.rviz')
    except:
        # フォールバック：設定ファイルが見つからない場合はデフォルト
        rviz_config_file = None
    
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
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    launch_image_view_arg = DeclareLaunchArgument(
        'launch_image_view',
        default_value='false',
        description='Whether to launch image_view for annotated images'
    )
    
    # Static transform for camera frame
    static_transform_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_link'],
        output='screen'
    )
    
    # 顔検出システムノード
    face_detection_node = Node(
        package='susumu_face_engagement_detector',
        executable='face_detection_node',
        name='face_detection_node',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'detection_model': LaunchConfiguration('detection_model')
        }],
        output='screen'
    )
    
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
    
    gaze_analysis_node = Node(
        package='susumu_face_engagement_detector',
        executable='gaze_analysis_node',
        name='gaze_analysis_node',
        parameters=[{
            'gaze_threshold_px': 50,
            'gaze_duration': 2.0
        }],
        output='screen'
    )
    
    engagement_manager_node = Node(
        package='susumu_face_engagement_detector',
        executable='engagement_manager_node',
        name='engagement_manager_node',
        parameters=[{
            'face_timeout': 1.0
        }],
        output='screen'
    )
    
    # RViz2ノード
    nodes = [
        image_topic_arg,
        detection_model_arg,
        launch_rviz_arg,
        launch_image_view_arg,
        static_transform_pub,
        face_detection_node,
        face_recognition_node,
        gaze_analysis_node,
        engagement_manager_node
    ]
    
    # Image Viewノード（条件付き）
    # Note: image_view は GUI環境でのみ動作するため、launch_image_view:=true で明示的に有効化
    # 例: ros2 launch ... launch_image_view:=true
    
    # RVizノードを条件付きで追加
    if rviz_config_file and os.path.exists(rviz_config_file):
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    else:
        # 設定ファイルなしでRViz起動
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    
    nodes.append(rviz_node)
    
    return LaunchDescription(nodes)