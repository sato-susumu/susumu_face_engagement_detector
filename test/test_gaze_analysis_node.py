import pytest
import time
from unittest.mock import patch

import rclpy
from std_msgs.msg import String

from susumu_face_engagement_detector.gaze_analysis_node import GazeAnalysisNode, GazeDetector


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def gaze_analysis_node(rclpy_init):
    node = GazeAnalysisNode()
    yield node
    node.destroy_node()


class TestGazeAnalysisNode:
    def test_node_initialization(self, gaze_analysis_node):
        assert gaze_analysis_node.get_name() == 'gaze_analysis_node'
        assert gaze_analysis_node._gazer is not None

    def test_parameter_configuration(self, rclpy_init):
        node = GazeAnalysisNode()
        
        # Test default parameters
        assert node.get_parameter('gaze_threshold_px').value == 50
        assert node.get_parameter('gaze_duration').value == 2.0
        
        node.destroy_node()


class TestGazeDetector:
    def test_gaze_detector_initialization(self):
        detector = GazeDetector(threshold_px=50, duration=2.0)
        assert detector._threshold == 50
        assert detector._duration == 2.0
        assert len(detector._start_times) == 0
        assert len(detector._state) == 0

    def test_face_looking_at_center(self):
        detector = GazeDetector(threshold_px=50, duration=2.0)
        
        # Test face looking at center (engaged)
        with patch('time.time', return_value=0.0):
            result = detector.check(320, 640, 'user_1', 0.0)  # center of 640px frame
            assert result is None  # Not enough time passed
        
        # Test after duration
        with patch('time.time', return_value=2.1):
            result = detector.check(320, 640, 'user_1', 2.1)
            assert result is True  # Should be engaged

    def test_face_looking_away(self):
        detector = GazeDetector(threshold_px=50, duration=2.0)
        
        # First engage the face
        with patch('time.time', return_value=0.0):
            detector.check(320, 640, 'user_1', 0.0)
        with patch('time.time', return_value=2.1):
            detector.check(320, 640, 'user_1', 2.1)
        
        # Test face looking away
        with patch('time.time', return_value=3.0):
            result = detector.check(100, 640, 'user_1', 3.0)  # far from center
            assert result is False  # Should be disengaged

    def test_multiple_faces(self):
        detector = GazeDetector(threshold_px=50, duration=2.0)
        
        # Test multiple faces independently
        with patch('time.time', return_value=0.0):
            result1 = detector.check(320, 640, 'user_1', 0.0)
            result2 = detector.check(100, 640, 'user_2', 0.0)
            assert result1 is None
            assert result2 is None
        
        # After duration, only centered face should be engaged
        with patch('time.time', return_value=2.1):
            result1 = detector.check(320, 640, 'user_1', 2.1)
            result2 = detector.check(100, 640, 'user_2', 2.1)
            assert result1 is True
            assert result2 is None

    def test_reset_face(self):
        detector = GazeDetector(threshold_px=50, duration=2.0)
        
        # Engage a face
        with patch('time.time', return_value=0.0):
            detector.check(320, 640, 'user_1', 0.0)
        with patch('time.time', return_value=2.1):
            detector.check(320, 640, 'user_1', 2.1)
        
        # Reset the face
        was_engaged = detector.reset_face('user_1')
        assert was_engaged is True
        assert 'user_1' not in detector._state
        assert 'user_1' not in detector._start_times


class TestGazeAnalysisNodeIntegration:
    def test_face_identity_processing(self, gaze_analysis_node):
        received_messages = []
        
        def message_callback(msg):
            received_messages.append(msg.data)
        
        # Create subscription to capture published messages
        gaze_analysis_node.create_subscription(
            String,
            'gaze_status',
            message_callback,
            10
        )

        # Create test identity message (face at center)
        identity_data = "user_1|320|240|40|60|640|480"  # center of frame
        identity_msg = String(data=identity_data)

        # Mock time for consistent testing
        with patch('time.time', return_value=1.0):
            # Process identity message
            gaze_analysis_node._on_face_identity(identity_msg)
            
            # Spin once to process callbacks
            rclpy.spin_once(gaze_analysis_node, timeout_sec=0.1)
            
            # Check if gaze status message was published
            assert len(received_messages) > 0
            
            # Parse the published message
            data_parts = received_messages[0].split('|')
            assert data_parts[0] == 'user_1'
            assert data_parts[1] in ['TRACKING', 'ENGAGED', 'DISENGAGED']

    def test_engagement_sequence(self, gaze_analysis_node):
        received_messages = []
        
        def message_callback(msg):
            received_messages.append(msg.data)
        
        # Create subscription to capture published messages
        gaze_analysis_node.create_subscription(
            String,
            'gaze_status',
            message_callback,
            10
        )

        # Test sequence: tracking -> engaged -> disengaged
        identity_data = "user_1|320|240|40|60|640|480"
        
        # First message - should be TRACKING
        with patch('time.time', return_value=0.0):
            gaze_analysis_node._on_face_identity(String(data=identity_data))
            rclpy.spin_once(gaze_analysis_node, timeout_sec=0.1)
        
        # Second message after duration - should be ENGAGED
        with patch('time.time', return_value=2.1):
            gaze_analysis_node._on_face_identity(String(data=identity_data))
            rclpy.spin_once(gaze_analysis_node, timeout_sec=0.1)
        
        # Third message with face looking away - should be DISENGAGED
        off_center_data = "user_1|100|240|40|60|640|480"  # far from center
        with patch('time.time', return_value=3.0):
            gaze_analysis_node._on_face_identity(String(data=off_center_data))
            rclpy.spin_once(gaze_analysis_node, timeout_sec=0.1)
        
        # Should have received multiple status updates
        assert len(received_messages) >= 3

    def test_invalid_identity_message(self, gaze_analysis_node):
        received_messages = []
        
        def message_callback(msg):
            received_messages.append(msg.data)
        
        gaze_analysis_node.create_subscription(
            String,
            'gaze_status',
            message_callback,
            10
        )

        # Send invalid message (too few parts)
        invalid_msg = String(data="incomplete|data")
        gaze_analysis_node._on_face_identity(invalid_msg)
        
        rclpy.spin_once(gaze_analysis_node, timeout_sec=0.1)
        
        # Should not publish anything for invalid message
        assert len(received_messages) == 0

    def test_edge_case_positions(self, gaze_analysis_node):
        received_messages = []
        
        def message_callback(msg):
            received_messages.append(msg.data)
        
        gaze_analysis_node.create_subscription(
            String,
            'gaze_status',
            message_callback,
            10
        )

        # Test edge positions
        test_cases = [
            "user_1|0|240|40|60|640|480",      # far left
            "user_1|640|240|40|60|640|480",    # far right
            "user_1|270|240|40|60|640|480",    # just within threshold
            "user_1|370|240|40|60|640|480",    # just within threshold
        ]

        for i, test_data in enumerate(test_cases):
            with patch('time.time', return_value=float(i)):
                gaze_analysis_node._on_face_identity(String(data=test_data))
                rclpy.spin_once(gaze_analysis_node, timeout_sec=0.1)

        # Should have processed all messages
        assert len(received_messages) == len(test_cases)


@pytest.mark.integration
def test_gaze_analysis_pipeline(rclpy_init):
    """Integration test for the complete gaze analysis pipeline"""
    node = GazeAnalysisNode()
    
    # Test multiple faces at different positions
    test_identities = [
        "user_1|320|240|40|60|640|480",  # center - should engage
        "user_2|100|200|50|70|640|480",  # off-center - should not engage
        "user_3|540|300|30|50|640|480",  # off-center - should not engage
    ]
    
    for i, identity_data in enumerate(test_identities):
        # This should not crash
        try:
            with patch('time.time', return_value=float(i)):
                node._on_face_identity(String(data=identity_data))
            assert True
        except Exception as e:
            pytest.fail(f"Gaze analysis processing failed: {e}")
    
    node.destroy_node()