from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
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
                    'image_topic': '/image',
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

    return LaunchDescription([container])