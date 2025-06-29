import pytest
import time
from unittest.mock import patch

import rclpy
from std_msgs.msg import String

from susumu_face_engagement_detector.engagement_manager_node import EngagementManagerNode


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def engagement_manager_node(rclpy_init):
    node = EngagementManagerNode()
    yield node
    node.destroy_node()


class TestEngagementManagerNode:
    def test_node_initialization(self, engagement_manager_node):
        assert engagement_manager_node.get_name() == 'engagement_manager_node'
        assert engagement_manager_node._active_faces is not None
        assert engagement_manager_node._last_seen is not None
        assert engagement_manager_node._engagement_state is not None

    def test_parameter_configuration(self, rclpy_init):
        node = EngagementManagerNode()
        
        # Test default parameters
        assert node.get_parameter('face_timeout').value == 1.0
        
        node.destroy_node()


class TestEngagementManagerNodeIntegration:
    def test_face_detection_event(self, engagement_manager_node):
        face_messages = []
        gaze_messages = []
        
        def face_event_callback(msg):
            face_messages.append(msg.data)
        
        def gaze_event_callback(msg):
            gaze_messages.append(msg.data)
        
        # Create subscriptions to capture published messages
        engagement_manager_node.create_subscription(
            String,
            'face_event',
            face_event_callback,
            10
        )
        engagement_manager_node.create_subscription(
            String,
            'gaze_event',
            gaze_event_callback,
            10
        )

        # Test first gaze status (should trigger face detection)
        gaze_status = "user_1|TRACKING|320|240|40|60"
        gaze_msg = String(data=gaze_status)

        with patch('time.time', return_value=1.0):
            engagement_manager_node._on_gaze_status(gaze_msg)
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)

        # Should have published face DETECTED event
        assert len(face_messages) > 0
        assert face_messages[0] == 'user_1:DETECTED'

    def test_engagement_events(self, engagement_manager_node):
        face_messages = []
        gaze_messages = []
        
        def face_event_callback(msg):
            face_messages.append(msg.data)
        
        def gaze_event_callback(msg):
            gaze_messages.append(msg.data)
        
        # Create subscriptions
        engagement_manager_node.create_subscription(
            String,
            'face_event',
            face_event_callback,
            10
        )
        engagement_manager_node.create_subscription(
            String,
            'gaze_event',
            gaze_event_callback,
            10
        )

        # First, send TRACKING status
        with patch('time.time', return_value=1.0):
            tracking_msg = String(data="user_1|TRACKING|320|240|40|60")
            engagement_manager_node._on_gaze_status(tracking_msg)
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)

        # Then send ENGAGED status
        with patch('time.time', return_value=2.0):
            engaged_msg = String(data="user_1|ENGAGED|320|240|40|60")
            engagement_manager_node._on_gaze_status(engaged_msg)
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)

        # Then send DISENGAGED status
        with patch('time.time', return_value=3.0):
            disengaged_msg = String(data="user_1|DISENGAGED|320|240|40|60")
            engagement_manager_node._on_gaze_status(disengaged_msg)
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)

        # Should have received engagement events
        engagement_events = [msg for msg in gaze_messages if 'ENGAGED' in msg or 'DISENGAGED' in msg]
        assert len(engagement_events) >= 2

    def test_face_timeout(self, engagement_manager_node):
        face_messages = []
        
        def face_event_callback(msg):
            face_messages.append(msg.data)
        
        # Create subscriptions
        engagement_manager_node.create_subscription(
            String,
            'face_event',
            face_event_callback,
            10
        )

        # Add a face
        with patch('time.time', return_value=1.0):
            tracking_msg = String(data="user_1|TRACKING|320|240|40|60")
            engagement_manager_node._on_gaze_status(tracking_msg)
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)

        # Clear previous messages
        face_messages.clear()

        # Simulate timeout check after timeout duration
        with patch('time.time', return_value=3.0):  # Beyond timeout
            engagement_manager_node._check_face_timeout()
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)

        # Should have published LOST event
        lost_events = [msg for msg in face_messages if 'LOST' in msg]
        assert len(lost_events) > 0
        assert lost_events[0] == 'user_1:LOST'

    def test_multiple_faces(self, engagement_manager_node):
        face_messages = []
        
        def face_event_callback(msg):
            face_messages.append(msg.data)
        
        # Create subscriptions
        engagement_manager_node.create_subscription(
            String,
            'face_event',
            face_event_callback,
            10
        )

        # Add multiple faces
        with patch('time.time', return_value=1.0):
            engagement_manager_node._on_gaze_status(String(data="user_1|TRACKING|320|240|40|60"))
            engagement_manager_node._on_gaze_status(String(data="user_2|TRACKING|400|300|50|70"))
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)

        # Should have detected both faces
        detected_events = [msg for msg in face_messages if 'DETECTED' in msg]
        assert len(detected_events) == 2
        assert 'user_1:DETECTED' in detected_events
        assert 'user_2:DETECTED' in detected_events

    def test_engagement_state_tracking(self, engagement_manager_node):
        gaze_messages = []
        
        def gaze_event_callback(msg):
            gaze_messages.append(msg.data)
        
        engagement_manager_node.create_subscription(
            String,
            'gaze_event',
            gaze_event_callback,
            10
        )

        # Test engagement state transitions
        with patch('time.time', return_value=1.0):
            # First ENGAGED event
            engagement_manager_node._on_gaze_status(String(data="user_1|ENGAGED|320|240|40|60"))
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)
            
            # Second ENGAGED event (should not trigger duplicate)
            engagement_manager_node._on_gaze_status(String(data="user_1|ENGAGED|320|240|40|60"))
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)

        # Should only have one ENGAGED event
        engaged_events = [msg for msg in gaze_messages if 'ENGAGED' in msg]
        assert len(engaged_events) == 1

    def test_invalid_gaze_status_message(self, engagement_manager_node):
        face_messages = []
        
        def face_event_callback(msg):
            face_messages.append(msg.data)
        
        engagement_manager_node.create_subscription(
            String,
            'face_event',
            face_event_callback,
            10
        )

        # Send invalid message (too few parts)
        invalid_msg = String(data="incomplete")
        engagement_manager_node._on_gaze_status(invalid_msg)
        
        rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)
        
        # Should not publish anything for invalid message
        assert len(face_messages) == 0

    def test_face_reappearance(self, engagement_manager_node):
        face_messages = []
        
        def face_event_callback(msg):
            face_messages.append(msg.data)
        
        engagement_manager_node.create_subscription(
            String,
            'face_event',
            face_event_callback,
            10
        )

        # Face appears
        with patch('time.time', return_value=1.0):
            engagement_manager_node._on_gaze_status(String(data="user_1|TRACKING|320|240|40|60"))
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)

        # Face times out
        with patch('time.time', return_value=3.0):
            engagement_manager_node._check_face_timeout()
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)

        # Face reappears
        with patch('time.time', return_value=4.0):
            engagement_manager_node._on_gaze_status(String(data="user_1|TRACKING|320|240|40|60"))
            rclpy.spin_once(engagement_manager_node, timeout_sec=0.1)

        # Should have DETECTED, LOST, DETECTED sequence
        detected_events = [msg for msg in face_messages if 'DETECTED' in msg]
        lost_events = [msg for msg in face_messages if 'LOST' in msg]
        assert len(detected_events) == 2
        assert len(lost_events) == 1


@pytest.mark.integration
def test_engagement_manager_pipeline(rclpy_init):
    """Integration test for the complete engagement management pipeline"""
    node = EngagementManagerNode()
    
    # Test complete workflow with multiple faces and states
    test_scenarios = [
        ("user_1", "TRACKING"),
        ("user_1", "ENGAGED"),
        ("user_2", "TRACKING"),
        ("user_1", "DISENGAGED"),
        ("user_2", "ENGAGED"),
    ]
    
    for i, (face_id, status) in enumerate(test_scenarios):
        gaze_data = f"{face_id}|{status}|320|240|40|60"
        
        # This should not crash
        try:
            with patch('time.time', return_value=float(i)):
                node._on_gaze_status(String(data=gaze_data))
            assert True
        except Exception as e:
            pytest.fail(f"Engagement management processing failed: {e}")
    
    # Test timeout mechanism
    try:
        with patch('time.time', return_value=10.0):
            node._check_face_timeout()
        assert True
    except Exception as e:
        pytest.fail(f"Face timeout check failed: {e}")
    
    node.destroy_node()