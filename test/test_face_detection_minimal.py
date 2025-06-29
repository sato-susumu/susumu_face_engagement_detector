import pytest
import sys
from unittest.mock import MagicMock, patch, Mock

# Mock face_recognition before any imports
sys.modules['face_recognition'] = MagicMock()
sys.modules['cv2'] = MagicMock()

import rclpy
from std_msgs.msg import String


def test_face_detection_imports():
    """Test that we can import face detection modules with mocked dependencies"""
    try:
        from susumu_face_engagement_detector.face_detection_node import FaceDetectionNode, FaceDetector
        assert True
    except ImportError as e:
        pytest.fail(f"Failed to import face detection modules: {e}")


def test_face_recognition_imports():
    """Test that we can import face recognition modules with mocked dependencies"""
    try:
        from susumu_face_engagement_detector.face_recognition_node import FaceRecognitionNode, FaceIdentifier
        assert True
    except ImportError as e:
        pytest.fail(f"Failed to import face recognition modules: {e}")


def test_gaze_analysis_imports():
    """Test that we can import gaze analysis modules"""
    try:
        from susumu_face_engagement_detector.gaze_analysis_node import GazeAnalysisNode, GazeDetector
        assert True
    except ImportError as e:
        pytest.fail(f"Failed to import gaze analysis modules: {e}")


def test_engagement_manager_imports():
    """Test that we can import engagement manager modules"""
    try:
        from susumu_face_engagement_detector.engagement_manager_node import EngagementManagerNode
        assert True
    except ImportError as e:
        pytest.fail(f"Failed to import engagement manager modules: {e}")


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_gaze_detector_functionality():
    """Test GazeDetector without external dependencies"""
    from susumu_face_engagement_detector.gaze_analysis_node import GazeDetector
    
    detector = GazeDetector(threshold_px=50, duration=2.0)
    assert detector._threshold == 50
    assert detector._duration == 2.0
    assert len(detector._start_times) == 0
    assert len(detector._state) == 0


def test_basic_node_creation(rclpy_init):
    """Test that nodes can be created with mocked dependencies"""
    from susumu_face_engagement_detector.gaze_analysis_node import GazeAnalysisNode
    from susumu_face_engagement_detector.engagement_manager_node import EngagementManagerNode
    
    # Test GazeAnalysisNode
    gaze_node = GazeAnalysisNode()
    assert gaze_node.get_name() == 'gaze_analysis_node'
    gaze_node.destroy_node()
    
    # Test EngagementManagerNode
    manager_node = EngagementManagerNode()
    assert manager_node.get_name() == 'engagement_manager_node'
    manager_node.destroy_node()


def test_string_message_processing():
    """Test basic message processing without external dependencies"""
    from susumu_face_engagement_detector.gaze_analysis_node import GazeAnalysisNode
    
    node = GazeAnalysisNode()
    
    # Test invalid message handling
    invalid_msg = String(data="incomplete|data")
    
    # This should not crash
    try:
        node._on_face_identity(invalid_msg)
        assert True
    except Exception as e:
        pytest.fail(f"Message processing failed: {e}")
    
    node.destroy_node()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])