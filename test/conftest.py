import pytest
import rclpy


@pytest.fixture(scope="session", autouse=True)
def rclpy_session():
    """Session-wide ROS2 initialization and cleanup"""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def ros_context():
    """Per-test ROS2 context"""
    # This can be used for tests that need a fresh context
    pass