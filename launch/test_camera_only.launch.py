from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """合成カメラのみのテスト用ランチファイル"""
    
    # Launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Camera image topic name'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='10.0',
        description='Synthetic camera FPS'
    )
    
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='synthetic',
        description='Test mode: synthetic or file'
    )
    
    image_file_arg = DeclareLaunchArgument(
        'image_file',
        default_value='test_images/multiple_faces_test.jpg',
        description='Test image file path (when test_mode=file)'
    )
    
    width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Image width'
    )
    
    height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Image height'
    )
    
    return LaunchDescription([
        image_topic_arg,
        fps_arg,
        test_mode_arg,
        image_file_arg,
        width_arg,
        height_arg,
        
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
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height')
            }],
            output='screen'
        )
    ])