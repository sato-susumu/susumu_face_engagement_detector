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
    
    return LaunchDescription([
        image_topic_arg,
        Node(
            package='susumu_face_engagement_detector',
            executable='face_detection_node',
            name='face_detection_node',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'detection_model': 'hog'
            }],
            output='screen'
        ),
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