"""Launch the engagement pipeline: detection + head pose + optional gaze + expression + engagement."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    image_topic = LaunchConfiguration('image_topic')
    detection_backend = LaunchConfiguration('detection_backend')
    detection_model = LaunchConfiguration('detection_model_path')
    gaze_enabled = LaunchConfiguration('enable_gaze')
    gaze_model = LaunchConfiguration('gaze_model_path')
    gaze_device = LaunchConfiguration('gaze_device')
    detection_score_threshold = ParameterValue(
        LaunchConfiguration('detection_score_threshold'),
        value_type=float,
    )

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument(
            'detection_backend',
            default_value='yunet',
            description='dlib_hog | dlib_cnn | yunet',
        ),
        DeclareLaunchArgument(
            'detection_model_path',
            default_value=os.path.expanduser('~/models/face_detection/face_detection_yunet_2023mar.onnx'),
            description='Path to detector weights (required for YuNet/SCRFD)',
        ),
        DeclareLaunchArgument(
            'detection_score_threshold',
            default_value='0.8',
            description='Minimum detector confidence accepted as a face',
        ),
        DeclareLaunchArgument(
            'enable_gaze',
            default_value='false',
            description='Start OpenVINO gaze_node when true',
        ),
        DeclareLaunchArgument(
            'gaze_model_path',
            default_value=os.path.expanduser(
                '~/models/gaze_estimation/intel/gaze-estimation-adas-0002/FP32/'
                'gaze-estimation-adas-0002.xml'
            ),
            description='Path to OpenVINO gaze-estimation-adas-0002 XML',
        ),
        DeclareLaunchArgument(
            'gaze_device',
            default_value='CPU',
            description='OpenVINO device for gaze inference',
        ),

        Node(
            package='susumu_face_engagement_detector',
            executable='face_detection_node',
            name='face_detection_node',
            parameters=[{
                'image_topic': image_topic,
                'detection_backend': detection_backend,
                'model_path': detection_model,
                'score_threshold': detection_score_threshold,
                'downsample_factor': 1.0,
                'max_fps': 0.0,
            }],
        ),
        Node(
            package='susumu_face_engagement_detector',
            executable='head_pose_node',
            name='head_pose_node',
            parameters=[{'image_topic': image_topic}],
        ),
        Node(
            package='susumu_face_engagement_detector',
            executable='gaze_node',
            name='gaze_node',
            condition=IfCondition(gaze_enabled),
            parameters=[{
                'image_topic': image_topic,
                'gaze_model_path': gaze_model,
                'gaze_device': gaze_device,
            }],
        ),
        Node(
            package='susumu_face_engagement_detector',
            executable='expression_node',
            name='expression_node',
            parameters=[{
                'image_topic': image_topic,
                'detection_backend': detection_backend,
                'detection_model_path': detection_model,
                'score_threshold': detection_score_threshold,
            }],
        ),
        Node(
            package='susumu_face_engagement_detector',
            executable='engagement_node',
            name='engagement_node',
            parameters=[{'update_hz': 10.0}],
        ),
    ])
