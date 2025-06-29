import pytest
import sys
from unittest.mock import MagicMock

# Mock external dependencies
sys.modules['face_recognition'] = MagicMock()
sys.modules['cv2'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()


def test_imports_work():
    """Test that all modules can be imported with mocked dependencies"""
    try:
        from susumu_face_engagement_detector.face_detection_node import FaceDetector
        from susumu_face_engagement_detector.face_recognition_node import FaceIdentifier  
        from susumu_face_engagement_detector.gaze_analysis_node import GazeDetector
        assert True
    except ImportError as e:
        pytest.fail(f"Import failed: {e}")


def test_gaze_detector_basic():
    """Test GazeDetector basic functionality"""
    from susumu_face_engagement_detector.gaze_analysis_node import GazeDetector
    
    detector = GazeDetector(threshold_px=50, duration=2.0)
    
    # Test initialization
    assert detector._threshold == 50
    assert detector._duration == 2.0
    assert isinstance(detector._start_times, dict)
    assert isinstance(detector._state, dict)
    
    # Test reset functionality
    detector._state['test_user'] = True
    detector._start_times['test_user'] = 1.0
    
    was_engaged = detector.reset_face('test_user')
    assert was_engaged is True
    assert 'test_user' not in detector._state
    assert 'test_user' not in detector._start_times


def test_face_detector_mock():
    """Test FaceDetector with mocked face_recognition"""
    from susumu_face_engagement_detector.face_detection_node import FaceDetector
    import numpy as np
    
    # Mock face_recognition functions
    sys.modules['face_recognition'].face_locations.return_value = [(30, 70, 70, 30)]
    sys.modules['face_recognition'].face_encodings.return_value = [np.random.rand(128)]
    
    detector = FaceDetector('hog')
    test_image = np.zeros((100, 100, 3), dtype=np.uint8)
    
    locations, encodings = detector.detect(test_image)
    
    assert len(locations) == 1
    assert len(encodings) == 1
    assert locations[0] == (30, 70, 70, 30)


def test_face_identifier_basic():
    """Test FaceIdentifier basic functionality with mocks"""
    from susumu_face_engagement_detector.face_recognition_node import FaceIdentifier
    import numpy as np
    
    # Mock os.path.exists to return False (no known faces directory)
    with pytest.MonkeyPatch().context() as m:
        m.setattr('os.path.exists', lambda x: False)
        
        identifier = FaceIdentifier('test_dir', 0.6)
        
        # Test new face identification
        test_encoding = np.random.rand(128)
        face_id = identifier.identify(test_encoding)
        
        assert face_id == 'user_1'
        assert len(identifier._tracked_ids) == 1
        assert len(identifier._tracked_encodings) == 1


def test_message_format_parsing():
    """Test message format parsing without ROS dependencies"""
    # Test face detection message format
    detection_data = "0|100|50|40|60|640|480|0.1,0.2,0.3"
    parts = detection_data.split('|')
    
    assert len(parts) == 8
    assert parts[0] == '0'  # face index
    assert parts[1] == '100'  # center_x
    assert parts[2] == '50'   # center_y
    assert parts[7] == '0.1,0.2,0.3'  # encoding
    
    # Test identity message format
    identity_data = "user_1|320|240|40|60|640|480"
    parts = identity_data.split('|')
    
    assert len(parts) == 7
    assert parts[0] == 'user_1'  # face_id
    assert parts[1] == '320'     # center_x
    
    # Test gaze status format
    gaze_data = "user_1|ENGAGED|320|240|40|60"
    parts = gaze_data.split('|')
    
    assert len(parts) == 6
    assert parts[0] == 'user_1'
    assert parts[1] == 'ENGAGED'


if __name__ == '__main__':
    pytest.main([__file__, '-v'])