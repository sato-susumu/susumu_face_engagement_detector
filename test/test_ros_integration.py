import pytest
import sys
from unittest.mock import MagicMock, patch
import time

# Mock external dependencies before importing ROS modules
sys.modules['face_recognition'] = MagicMock()
sys.modules['cv2'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()

import rclpy
from std_msgs.msg import String


@pytest.fixture(scope="session")
def rclpy_session():
    """Session-wide ROS2 initialization"""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_gaze_analysis_node_creation(rclpy_session):
    """Test GazeAnalysisNode can be created and destroyed"""
    from susumu_face_engagement_detector.gaze_analysis_node import GazeAnalysisNode
    
    node = GazeAnalysisNode()
    assert node.get_name() == 'gaze_analysis_node'
    
    # Test parameter access
    threshold = node.get_parameter('gaze_threshold_px').value
    duration = node.get_parameter('gaze_duration').value
    assert threshold == 50
    assert duration == 2.0
    
    node.destroy_node()


def test_engagement_manager_node_creation(rclpy_session):
    """Test EngagementManagerNode can be created and destroyed"""
    from susumu_face_engagement_detector.engagement_manager_node import EngagementManagerNode
    
    node = EngagementManagerNode()
    assert node.get_name() == 'engagement_manager_node'
    
    # Test parameter access
    timeout = node.get_parameter('face_timeout').value
    assert timeout == 1.0
    
    node.destroy_node()


def test_face_detection_node_creation(rclpy_session):
    """Test FaceDetectionNode can be created with mocked dependencies"""
    from susumu_face_engagement_detector.face_detection_node import FaceDetectionNode
    
    # Mock cv_bridge
    with patch('susumu_face_engagement_detector.face_detection_node.CvBridge'):
        node = FaceDetectionNode()
        assert node.get_name() == 'face_detection_node'
        
        # Test parameters
        topic = node.get_parameter('image_topic').value
        model = node.get_parameter('detection_model').value
        assert topic == '/image'
        assert model == 'hog'
        
        node.destroy_node()


def test_face_recognition_node_creation(rclpy_session):
    """Test FaceRecognitionNode can be created with mocked dependencies"""
    from susumu_face_engagement_detector.face_recognition_node import FaceRecognitionNode
    
    with patch('os.path.exists', return_value=False):
        node = FaceRecognitionNode()
        assert node.get_name() == 'face_recognition_node'
        
        # Test parameters
        tolerance = node.get_parameter('match_tolerance').value
        known_dir = node.get_parameter('known_faces_dir').value
        assert tolerance == 0.6
        assert known_dir == 'known_faces'
        
        node.destroy_node()


def test_message_communication(rclpy_session):
    """Test basic message communication between nodes"""
    from susumu_face_engagement_detector.gaze_analysis_node import GazeAnalysisNode
    from susumu_face_engagement_detector.engagement_manager_node import EngagementManagerNode
    
    gaze_node = GazeAnalysisNode()
    manager_node = EngagementManagerNode()
    
    received_messages = []
    
    def callback(msg):
        received_messages.append(msg.data)
    
    # Create subscription
    manager_node.create_subscription(String, 'face_event', callback, 10)
    
    # Simulate gaze status message
    gaze_msg = String(data="user_1|TRACKING|320|240|40|60")
    
    with patch('time.time', return_value=1.0):
        manager_node._on_gaze_status(gaze_msg)
        
        # Process callbacks
        rclpy.spin_once(manager_node, timeout_sec=0.1)
    
    # Check if message was processed
    assert len(received_messages) > 0
    assert 'user_1:DETECTED' in received_messages[0]
    
    gaze_node.destroy_node()
    manager_node.destroy_node()


def test_gaze_detection_logic(rclpy_session):
    """Test gaze detection logic without external dependencies"""
    from susumu_face_engagement_detector.gaze_analysis_node import GazeAnalysisNode
    
    node = GazeAnalysisNode()
    
    received_messages = []
    
    def callback(msg):
        received_messages.append(msg.data)
    
    node.create_subscription(String, 'gaze_status', callback, 10)
    
    # Test centered face (should eventually engage)
    identity_data = "user_1|320|240|40|60|640|480"  # Center of 640px frame
    
    # First message - should be TRACKING
    with patch('time.time', return_value=0.0):
        node._on_face_identity(String(data=identity_data))
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # After duration - should be ENGAGED  
    with patch('time.time', return_value=2.1):
        node._on_face_identity(String(data=identity_data))
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # Should have received status updates
    assert len(received_messages) >= 2
    
    # Check message format
    for msg in received_messages:
        parts = msg.split('|')
        assert len(parts) >= 2
        assert parts[0] == 'user_1'
        assert parts[1] in ['TRACKING', 'ENGAGED', 'DISENGAGED']
    
    node.destroy_node()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])