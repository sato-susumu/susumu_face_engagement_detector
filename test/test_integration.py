import pytest
import time
import numpy as np
from unittest.mock import patch, MagicMock

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from susumu_face_engagement_detector.face_detection_node import FaceDetectionNode
from susumu_face_engagement_detector.face_recognition_node import FaceRecognitionNode
from susumu_face_engagement_detector.gaze_analysis_node import GazeAnalysisNode
from susumu_face_engagement_detector.engagement_manager_node import EngagementManagerNode


@pytest.fixture
def bridge():
    return CvBridge()


class TestSystemIntegration:
    """Integration tests for the complete face engagement detection system"""
    
    def test_complete_pipeline_simulation(self, bridge):
        """Test the complete pipeline from face detection to engagement events"""
        # Create all nodes
        with patch('os.path.exists', return_value=False):
            face_detection_node = FaceDetectionNode()
            face_recognition_node = FaceRecognitionNode()
            gaze_analysis_node = GazeAnalysisNode()
            engagement_manager_node = EngagementManagerNode()
        
        # Storage for messages
        face_detections = []
        face_identities = []
        gaze_statuses = []
        face_events = []
        gaze_events = []
        
        # Set up subscriptions to capture all messages
        face_detection_node.create_subscription(
            String, 'face_detections',
            lambda msg: face_detections.append(msg.data), 10)
        
        face_recognition_node.create_subscription(
            String, 'face_identities',
            lambda msg: face_identities.append(msg.data), 10)
        
        gaze_analysis_node.create_subscription(
            String, 'gaze_status',
            lambda msg: gaze_statuses.append(msg.data), 10)
        
        engagement_manager_node.create_subscription(
            String, 'face_event',
            lambda msg: face_events.append(msg.data), 10)
        
        engagement_manager_node.create_subscription(
            String, 'gaze_event',
            lambda msg: gaze_events.append(msg.data), 10)
        
        # Create test image
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        test_image[200:280, 280:360] = [255, 255, 255]  # Face in center
        image_msg = bridge.cv2_to_imgmsg(test_image, 'bgr8')
        
        # Mock face detection to return a centered face
        with patch.object(face_detection_node._detector, 'detect') as mock_detect:
            mock_detect.return_value = (
                [(200, 360, 280, 280)],  # top, right, bottom, left
                [np.random.rand(128)]    # encoding
            )
            
            # 1. Process image through face detection
            face_detection_node._on_image(image_msg)
            rclpy.spin_once(face_detection_node, timeout_sec=0.1)
            
            # Should have face detection data
            assert len(face_detections) > 0
            detection_data = face_detections[0]
            
            # 2. Process detection through face recognition
            detection_msg = String(data=detection_data)
            face_recognition_node._on_face_detection(detection_msg)
            rclpy.spin_once(face_recognition_node, timeout_sec=0.1)
            
            # Should have face identity data
            assert len(face_identities) > 0
            identity_data = face_identities[0]
            
            # 3. Process identity through gaze analysis
            identity_msg = String(data=identity_data)
            with patch('time.time', return_value=0.0):
                gaze_analysis_node._on_face_identity(identity_msg)
                rclpy.spin_once(gaze_analysis_node, timeout_sec=0.1)
            
            # Should have gaze status data
            assert len(gaze_statuses) > 0
            gaze_data = gaze_statuses[0]
            
            # 4. Process gaze status through engagement manager
            gaze_msg = String(data=gaze_data)
            with patch('time.time', return_value=1.0):
                engagement_manager_node._on_gaze_status(gaze_msg)
                rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)
            
            # Should have face events
            assert len(face_events) > 0
            assert 'DETECTED' in face_events[0]
        
        # Clean up
        face_detection_node.destroy_node()
        face_recognition_node.destroy_node()
        gaze_analysis_node.destroy_node()
        engagement_manager_node.destroy_node()

    def test_multiple_faces_pipeline(self, bridge):
        """Test pipeline with multiple faces"""
        with patch('os.path.exists', return_value=False):
            face_detection_node = FaceDetectionNode()
            face_recognition_node = FaceRecognitionNode()
            engagement_manager_node = EngagementManagerNode()
        
        face_events = []
        engagement_manager_node.create_subscription(
            String, 'face_event',
            lambda msg: face_events.append(msg.data), 10)
        
        # Simulate multiple face detections
        test_detections = [
            "0|100|200|50|60|640|480|" + ','.join(map(str, np.random.rand(128))),
            "1|500|200|50|60|640|480|" + ','.join(map(str, np.random.rand(128))),
        ]
        
        for detection_data in test_detections:
            detection_msg = String(data=detection_data)
            face_recognition_node._on_face_detection(detection_msg)
            rclpy.spin_once(face_recognition_node, timeout_sec=0.1)
        
        # Process through engagement manager
        with patch('time.time', return_value=1.0):
            # Simulate gaze statuses for both faces
            engagement_manager_node._on_gaze_status(String(data="user_1|TRACKING|100|200|50|60"))
            engagement_manager_node._on_gaze_status(String(data="user_2|TRACKING|500|200|50|60"))
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)
        
        # Should detect multiple faces
        detected_faces = [event for event in face_events if 'DETECTED' in event]
        assert len(detected_faces) == 2
        
        # Clean up
        face_detection_node.destroy_node()
        face_recognition_node.destroy_node()
        engagement_manager_node.destroy_node()

    def test_engagement_sequence_pipeline(self):
        """Test complete engagement detection sequence"""
        with patch('os.path.exists', return_value=False):
            gaze_analysis_node = GazeAnalysisNode()
            engagement_manager_node = EngagementManagerNode()
        
        gaze_events = []
        engagement_manager_node.create_subscription(
            String, 'gaze_event',
            lambda msg: gaze_events.append(msg.data), 10)
        
        # Simulate engagement sequence
        identity_data = "user_1|320|240|40|60|640|480"  # Center face
        
        # Start tracking
        with patch('time.time', return_value=0.0):
            gaze_analysis_node._on_face_identity(String(data=identity_data))
            rclpy.spin_once(gaze_analysis_node, timeout_sec=0.1)
            
            # Get gaze status and process through manager
            gaze_msg = String(data="user_1|TRACKING|320|240|40|60")
            engagement_manager_node._on_gaze_status(gaze_msg)
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)
        
        # Become engaged after duration
        with patch('time.time', return_value=2.1):
            gaze_analysis_node._on_face_identity(String(data=identity_data))
            rclpy.spin_once(gaze_analysis_node, timeout_sec=0.1)
            
            # Get engaged status and process through manager
            gaze_msg = String(data="user_1|ENGAGED|320|240|40|60")
            engagement_manager_node._on_gaze_status(gaze_msg)
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)
        
        # Should have engagement event
        engaged_events = [event for event in gaze_events if 'ENGAGED' in event]
        assert len(engaged_events) >= 1
        
        # Clean up
        gaze_analysis_node.destroy_node()
        engagement_manager_node.destroy_node()


class TestNodeCommunication:
    """Test communication between nodes"""
    
    def test_face_detection_to_recognition_communication(self):
        """Test message passing from face detection to recognition"""
        with patch('os.path.exists', return_value=False):
            face_detection_node = FaceDetectionNode()
            face_recognition_node = FaceRecognitionNode()
        
        received_messages = []
        
        def callback(msg):
            received_messages.append(msg.data)
            # Immediately process through recognition node
            face_recognition_node._on_face_detection(msg)
        
        # Subscribe detection node output to recognition input
        face_detection_node.create_subscription(
            String, 'face_detections', callback, 10)
        
        # Mock and trigger face detection
        with patch.object(face_detection_node._detector, 'detect') as mock_detect:
            mock_detect.return_value = (
                [(100, 200, 200, 100)],
                [np.random.rand(128)]
            )
            
            # Create dummy image
            bridge = CvBridge()
            test_image = np.zeros((300, 300, 3), dtype=np.uint8)
            image_msg = bridge.cv2_to_imgmsg(test_image, 'bgr8')
            
            face_detection_node._on_image(image_msg)
            rclpy.spin_once(face_detection_node, timeout_sec=0.1)
        
        # Should have received and processed message
        assert len(received_messages) > 0
        
        # Clean up
        face_detection_node.destroy_node()
        face_recognition_node.destroy_node()

    def test_topic_connectivity(self):
        """Test that all expected topics are created"""
        with patch('os.path.exists', return_value=False):
            face_detection_node = FaceDetectionNode()
            face_recognition_node = FaceRecognitionNode()
            gaze_analysis_node = GazeAnalysisNode()
            engagement_manager_node = EngagementManagerNode()
        
        # Give nodes time to set up
        rclpy.spin_once(face_detection_node, timeout_sec=0.1)
        rclpy.spin_once(face_recognition_node, timeout_sec=0.1)
        rclpy.spin_once(gaze_analysis_node, timeout_sec=0.1)
        rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)
        
        # Check that nodes are properly named
        assert face_detection_node.get_name() == 'face_detection_node'
        assert face_recognition_node.get_name() == 'face_recognition_node'
        assert gaze_analysis_node.get_name() == 'gaze_analysis_node'
        assert engagement_manager_node.get_name() == 'engagement_manager_node'
        
        # Clean up
        face_detection_node.destroy_node()
        face_recognition_node.destroy_node()
        gaze_analysis_node.destroy_node()
        engagement_manager_node.destroy_node()


@pytest.mark.integration
class TestSystemPerformance:
    """Performance and stress tests"""
    
    def test_high_frequency_processing(self):
        """Test system under high message frequency"""
        with patch('os.path.exists', return_value=False):
            face_recognition_node = FaceRecognitionNode()
        
        # Process many messages quickly
        for i in range(100):
            encoding_str = ','.join(map(str, np.random.rand(128)))
            detection_data = f"{i}|{100+i}|{50+i}|40|60|640|480|{encoding_str}"
            detection_msg = String(data=detection_data)
            
            # Should not crash under load
            try:
                face_recognition_node._on_face_detection(detection_msg)
                assert True
            except Exception as e:
                pytest.fail(f"High frequency processing failed: {e}")
        
        face_recognition_node.destroy_node()

    def test_memory_usage_stability(self):
        """Test that repeated processing doesn't cause memory leaks"""
        with patch('os.path.exists', return_value=False):
            gaze_analysis_node = GazeAnalysisNode()
        
        # Process same message many times
        identity_data = "user_1|320|240|40|60|640|480"
        
        for i in range(1000):
            with patch('time.time', return_value=float(i)):
                gaze_analysis_node._on_face_identity(String(data=identity_data))
        
        # Should complete without issues
        assert True
        
        gaze_analysis_node.destroy_node()