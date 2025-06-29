import pytest
import numpy as np
from unittest.mock import patch

import rclpy
from std_msgs.msg import String

from susumu_face_engagement_detector.face_recognition_node import FaceRecognitionNode, FaceIdentifier


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def face_recognition_node(rclpy_init):
    with patch('os.path.exists', return_value=False):
        node = FaceRecognitionNode()
    yield node
    node.destroy_node()


class TestFaceRecognitionNode:
    def test_node_initialization(self, face_recognition_node):
        assert face_recognition_node.get_name() == 'face_recognition_node'
        assert face_recognition_node._identifier is not None

    def test_parameter_configuration(self, rclpy_init):
        with patch('os.path.exists', return_value=False):
            node = FaceRecognitionNode()
            
            # Test default parameters
            assert node.get_parameter('known_faces_dir').value == 'known_faces'
            assert node.get_parameter('match_tolerance').value == 0.6
            
            node.destroy_node()


class TestFaceIdentifier:
    def test_face_identifier_new_face(self):
        with patch('os.path.exists', return_value=False):
            identifier = FaceIdentifier('test_dir', 0.6)
        
        # Test new face identification
        test_encoding = np.random.rand(128)
        face_id = identifier.identify(test_encoding)
        
        assert face_id == 'user_1'
        
        # Test same face recognition
        face_id2 = identifier.identify(test_encoding)
        assert face_id2 == 'user_1'

    def test_face_identifier_with_known_faces(self):
        with patch('os.path.exists', return_value=True), \
             patch('os.listdir', return_value=['john.jpg', 'jane.jpg']), \
             patch('face_recognition.load_image_file'), \
             patch('face_recognition.face_encodings') as mock_encodings:
            
            mock_encodings.return_value = [np.random.rand(128)]
            identifier = FaceIdentifier('known_faces', 0.6)
            
            assert len(identifier._known_ids) == 2
            assert 'john' in identifier._known_ids
            assert 'jane' in identifier._known_ids

    def test_face_identifier_known_face_priority(self):
        with patch('os.path.exists', return_value=True), \
             patch('os.listdir', return_value=['known_person.jpg']), \
             patch('face_recognition.load_image_file'), \
             patch('face_recognition.face_encodings') as mock_encodings, \
             patch('face_recognition.compare_faces') as mock_compare:
            
            known_encoding = np.random.rand(128)
            mock_encodings.return_value = [known_encoding]
            
            identifier = FaceIdentifier('known_faces', 0.6)
            
            # Mock that the test encoding matches the known face
            mock_compare.return_value = [True]
            
            test_encoding = np.random.rand(128)
            face_id = identifier.identify(test_encoding)
            
            assert face_id == 'known_person'

    def test_multiple_new_faces(self):
        with patch('os.path.exists', return_value=False):
            identifier = FaceIdentifier('test_dir', 0.6)
        
        # Add multiple different faces
        face1_encoding = np.random.rand(128)
        face2_encoding = np.random.rand(128)
        
        face_id1 = identifier.identify(face1_encoding)
        face_id2 = identifier.identify(face2_encoding)
        
        assert face_id1 == 'user_1'
        assert face_id2 == 'user_2'
        assert face_id1 != face_id2


class TestFaceRecognitionNodeIntegration:
    def test_face_detection_processing(self, face_recognition_node):
        received_messages = []
        
        def message_callback(msg):
            received_messages.append(msg.data)
        
        # Create subscription to capture published messages
        face_recognition_node.create_subscription(
            String,
            'face_identities',
            message_callback,
            10
        )

        # Create test detection message
        encoding_str = ','.join(map(str, np.random.rand(128)))
        detection_data = f"0|100|50|40|60|640|480|{encoding_str}"
        detection_msg = String(data=detection_data)

        # Mock face identification
        with patch.object(face_recognition_node._identifier, 'identify') as mock_identify:
            mock_identify.return_value = 'user_1'
            
            # Process detection message
            face_recognition_node._on_face_detection(detection_msg)
            
            # Spin once to process callbacks
            rclpy.spin_once(face_recognition_node, timeout_sec=0.1)
            
            # Check if identity message was published
            assert len(received_messages) > 0
            
            # Parse the published message
            data_parts = received_messages[0].split('|')
            assert len(data_parts) == 7
            assert data_parts[0] == 'user_1'  # face_id
            assert data_parts[1] == '100'     # center_x
            assert data_parts[2] == '50'      # center_y

    def test_invalid_detection_message(self, face_recognition_node):
        received_messages = []
        
        def message_callback(msg):
            received_messages.append(msg.data)
        
        face_recognition_node.create_subscription(
            String,
            'face_identities',
            message_callback,
            10
        )

        # Send invalid message (too few parts)
        invalid_msg = String(data="incomplete|data")
        face_recognition_node._on_face_detection(invalid_msg)
        
        rclpy.spin_once(face_recognition_node, timeout_sec=0.1)
        
        # Should not publish anything for invalid message
        assert len(received_messages) == 0

    def test_encoding_parsing(self, face_recognition_node):
        received_messages = []
        
        def message_callback(msg):
            received_messages.append(msg.data)
        
        face_recognition_node.create_subscription(
            String,
            'face_identities',
            message_callback,
            10
        )

        # Create test with specific encoding values
        test_encoding = [0.1, 0.2, 0.3]
        encoding_str = ','.join(map(str, test_encoding))
        detection_data = f"0|100|50|40|60|640|480|{encoding_str}"
        detection_msg = String(data=detection_data)

        with patch.object(face_recognition_node._identifier, 'identify') as mock_identify:
            mock_identify.return_value = 'test_user'
            
            face_recognition_node._on_face_detection(detection_msg)
            
            # Verify that identify was called with correct encoding
            mock_identify.assert_called_once()
            called_encoding = mock_identify.call_args[0][0]
            np.testing.assert_array_almost_equal(called_encoding, test_encoding)


@pytest.mark.integration
def test_face_recognition_pipeline(rclpy_init):
    """Integration test for the complete face recognition pipeline"""
    with patch('os.path.exists', return_value=False):
        node = FaceRecognitionNode()
    
    # Test multiple faces being processed
    encodings = [np.random.rand(3) for _ in range(3)]
    
    for i, encoding in enumerate(encodings):
        encoding_str = ','.join(map(str, encoding))
        detection_data = f"{i}|{100+i*10}|{50+i*5}|40|60|640|480|{encoding_str}"
        detection_msg = String(data=detection_data)
        
        # This should not crash
        try:
            node._on_face_detection(detection_msg)
            assert True
        except Exception as e:
            pytest.fail(f"Face recognition processing failed: {e}")
    
    node.destroy_node()