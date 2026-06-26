import pytest
import time
import numpy as np
import cv2
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from susumu_face_engagement_detector.face_detection_node import FaceDetectionNode, FaceDetector


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def face_detection_node(rclpy_init):
    node = FaceDetectionNode()
    yield node
    node.destroy_node()


@pytest.fixture
def bridge():
    return CvBridge()


class TestFaceDetectionNode:
    def test_node_initialization(self, face_detection_node):
        assert face_detection_node.get_name() == 'face_detection_node'
        assert face_detection_node._detector is not None
        assert face_detection_node._bridge is not None

    def test_face_detector(self):
        detector = FaceDetector('hog')
        
        # Create a test image with a simple face-like pattern
        test_image = np.zeros((100, 100, 3), dtype=np.uint8)
        test_image[30:70, 30:70] = [255, 255, 255]  # White square as mock face
        
        # Mock face_recognition functions
        with patch('face_recognition.face_locations') as mock_locations, \
             patch('face_recognition.face_encodings') as mock_encodings:
            
            mock_locations.return_value = [(30, 70, 70, 30)]  # top, right, bottom, left
            mock_encodings.return_value = [np.random.rand(128)]
            
            locations, encodings = detector.detect(test_image)
            
            assert len(locations) == 1
            assert len(encodings) == 1
            assert locations[0] == (30, 70, 70, 30)

    def test_image_processing(self, face_detection_node, bridge):
        received_messages = []
        
        def message_callback(msg):
            received_messages.append(msg.data)
        
        # Create subscription to capture published messages
        face_detection_node.create_subscription(
            String,
            'face_detections',
            message_callback,
            10
        )

        # Create test image
        test_image = np.zeros((200, 200, 3), dtype=np.uint8)
        test_image[50:150, 50:150] = [255, 255, 255]
        
        # Convert to ROS Image message
        image_msg = bridge.cv2_to_imgmsg(test_image, 'bgr8')

        # Mock face detection
        with patch.object(face_detection_node._detector, 'detect') as mock_detect:
            mock_detect.return_value = (
                [(50, 150, 150, 50)],  # locations
                [np.random.rand(128)]   # encodings
            )
            
            # Process image
            face_detection_node._on_image(image_msg)
            
            # Spin once to process callbacks
            rclpy.spin_once(face_detection_node, timeout_sec=0.1)
            
            # Check if message was published
            assert len(received_messages) > 0
            
            # Parse the published message
            data_parts = received_messages[0].split('|')
            assert len(data_parts) == 8  # idx, x, y, w, h, frame_w, frame_h, encoding
            assert data_parts[0] == '0'  # face index


class TestFaceDetectionNodeIntegration:
    def test_node_startup_and_shutdown(self, rclpy_init):
        node = FaceDetectionNode()
        
        # Test that node starts successfully
        assert node.get_name() == 'face_detection_node'
        
        # Test graceful shutdown
        node.destroy_node()

    def test_parameter_configuration(self, rclpy_init):
        node = FaceDetectionNode()
        
        # Test default parameters
        assert node.get_parameter('image_topic').value == '/image'
        assert node.get_parameter('detection_model').value == 'hog'
        
        node.destroy_node()


@pytest.mark.integration
def test_face_detection_with_real_image(rclpy_init):
    """Integration test with actual image processing"""
    node = FaceDetectionNode()
    bridge = CvBridge()
    
    # Create a more realistic test image
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    image_msg = bridge.cv2_to_imgmsg(test_image, 'bgr8')
    
    # This should not crash even with no faces detected
    try:
        node._on_image(image_msg)
        assert True  # Test passes if no exception is raised
    except Exception as e:
        pytest.fail(f"Image processing failed: {e}")
    
    node.destroy_node()