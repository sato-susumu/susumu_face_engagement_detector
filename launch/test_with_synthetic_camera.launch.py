from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """合成カメラとパイプライン全体のテスト用ランチファイル"""
    
    # Launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Camera image topic name'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='5.0',
        description='Synthetic camera FPS'
    )
    
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='synthetic',
        description='Test mode: synthetic or file'
    )
    
    image_file_arg = DeclareLaunchArgument(
        'image_file',
        default_value='test_images/person1_test.jpg',
        description='Test image file path (when test_mode=file)'
    )
    
    return LaunchDescription([
        image_topic_arg,
        fps_arg,
        test_mode_arg,
        image_file_arg,
        
        # 合成カメラノード
        Node(
            package='susumu_face_engagement_detector',
            executable='test_camera_node',
            name='test_camera_node',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'fps': LaunchConfiguration('fps'),
                'test_mode': LaunchConfiguration('test_mode'),
                'image_file': LaunchConfiguration('image_file'),
                'image_width': 640,
                'image_height': 480
            }],
            output='screen'
        ),
        
        # 顔検出ノード
        Node(
            package='susumu_face_engagement_detector',
            executable='face_detection_node',
            name='face_detection_node',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'detection_model': 'hog'  # CNNより高速
            }],
            output='screen'
        ),
        
        # 顔認識ノード
        Node(
            package='susumu_face_engagement_detector',
            executable='face_recognition_node',
            name='face_recognition_node',
            parameters=[{
                'known_faces_dir': 'known_faces',
                'match_tolerance': 0.6
            }],
            output='screen'
        ),
        
        # 注視分析ノード
        Node(
            package='susumu_face_engagement_detector',
            executable='gaze_analysis_node',
            name='gaze_analysis_node',
            parameters=[{
                'gaze_threshold_px': 50,
                'gaze_duration': 2.0
            }],
            output='screen'
        ),
        
        # エンゲージメント管理ノード
        Node(
            package='susumu_face_engagement_detector',
            executable='engagement_manager_node',
            name='engagement_manager_node',
            parameters=[{
                'face_timeout': 1.0
            }],
            output='screen'
        )
    ])