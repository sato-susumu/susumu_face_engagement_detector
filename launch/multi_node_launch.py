from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 起動引数定義
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Input image topic name'
    )
    container = ComposableNodeContainer(
        name='face_engagement_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='susumu_face_engagement_detector',
                plugin='susumu_face_engagement_detector.FaceDetectionNode',
                name='face_detection_node',
                parameters=[{
                    'image_topic': LaunchConfiguration('image_topic'),
                    'detection_model': 'hog'
                }]
            ),
            ComposableNode(
                package='susumu_face_engagement_detector',
                plugin='susumu_face_engagement_detector.FaceRecognitionNode',
                name='face_recognition_node',
                parameters=[{
                    'known_faces_dir': 'known_faces',
                    'match_tolerance': 0.6
                }]
            ),
            ComposableNode(
                package='susumu_face_engagement_detector',
                plugin='susumu_face_engagement_detector.GazeAnalysisNode',
                name='gaze_analysis_node',
                parameters=[{
                    'gaze_threshold_px': 50,
                    'gaze_duration': 2.0
                }]
            ),
            ComposableNode(
                package='susumu_face_engagement_detector',
                plugin='susumu_face_engagement_detector.EngagementManagerNode',
                name='engagement_manager_node',
                parameters=[{
                    'face_timeout': 1.0
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        image_topic_arg,
        container
    ])