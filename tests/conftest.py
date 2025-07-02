"""
Pytest configuration and shared fixtures for Duckietown testing.
"""

import pytest
import numpy as np
import cv2
import rospy
import os
import tempfile
from unittest.mock import Mock, MagicMock
from pathlib import Path


# Test configuration
TEST_DATA_DIR = Path(__file__).parent / "data"
MOCK_IMAGE_SIZE = (640, 480)


@pytest.fixture(scope="session")
def ros_environment():
    """Initialize ROS environment for testing."""
    try:
        rospy.init_node('test_node', anonymous=True)
        yield
    except rospy.exceptions.ROSException:
        # ROS already initialized or not available
        yield


@pytest.fixture
def mock_camera_info():
    """Mock camera info for testing."""
    camera_info = Mock()
    camera_info.height = MOCK_IMAGE_SIZE[1]
    camera_info.width = MOCK_IMAGE_SIZE[0]
    camera_info.distortion_model = "plumb_bob"
    camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info.K = [320.0, 0.0, 320.0, 0.0, 240.0, 240.0, 0.0, 0.0, 1.0]
    camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info.P = [320.0, 0.0, 320.0, 0.0, 0.0, 240.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    return camera_info


@pytest.fixture
def sample_image():
    """Generate a sample test image."""
    image = np.zeros((MOCK_IMAGE_SIZE[1], MOCK_IMAGE_SIZE[0], 3), dtype=np.uint8)
    # Add some lane-like features
    cv2.line(image, (100, 400), (200, 200), (255, 255, 255), 5)  # White lane
    cv2.line(image, (500, 400), (400, 200), (255, 255, 0), 5)    # Yellow lane
    return image


@pytest.fixture
def sample_image_with_duckie():
    """Generate a sample test image with a yellow duckie."""
    image = np.zeros((MOCK_IMAGE_SIZE[1], MOCK_IMAGE_SIZE[0], 3), dtype=np.uint8)
    # Add lane lines
    cv2.line(image, (100, 400), (200, 200), (255, 255, 255), 5)
    cv2.line(image, (500, 400), (400, 200), (255, 255, 0), 5)
    # Add yellow duckie (circle)
    cv2.circle(image, (320, 350), 30, (0, 255, 255), -1)  # Yellow in BGR
    return image


@pytest.fixture
def sample_image_with_led():
    """Generate a sample test image with LED patterns."""
    image = np.zeros((MOCK_IMAGE_SIZE[1], MOCK_IMAGE_SIZE[0], 3), dtype=np.uint8)
    # Add lane lines
    cv2.line(image, (100, 400), (200, 200), (255, 255, 255), 5)
    cv2.line(image, (500, 400), (400, 200), (255, 255, 0), 5)
    # Add white LEDs (front)
    cv2.circle(image, (300, 300), 8, (255, 255, 255), -1)
    cv2.circle(image, (340, 300), 8, (255, 255, 255), -1)
    # Add red LEDs (back)
    cv2.circle(image, (300, 350), 8, (0, 0, 255), -1)
    cv2.circle(image, (340, 350), 8, (0, 0, 255), -1)
    return image


@pytest.fixture
def mock_lane_pose():
    """Mock lane pose message."""
    lane_pose = Mock()
    lane_pose.d = 0.1  # 10cm to the right
    lane_pose.phi = 0.05  # 3 degrees
    lane_pose.in_lane = True
    lane_pose.status = "NORMAL"
    return lane_pose


@pytest.fixture
def mock_car_cmd():
    """Mock car command message."""
    car_cmd = Mock()
    car_cmd.v = 0.3  # 30cm/s
    car_cmd.omega = 0.0  # No rotation
    return car_cmd


@pytest.fixture
def mock_fsm_state():
    """Mock FSM state message."""
    fsm_state = Mock()
    fsm_state.state = "LANE_FOLLOWING"
    return fsm_state


@pytest.fixture
def temporary_config_file():
    """Create a temporary configuration file for testing."""
    config_content = """
# Test configuration
overtaking_steps: 4
step_duration: 2.0
lane_width: 0.22
emergency_stop_distance: 0.15
max_overtaking_distance: 1.0
min_detection_confidence: 0.7
"""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        f.write(config_content)
        f.flush()
        yield f.name
    os.unlink(f.name)


@pytest.fixture
def mock_object_detection():
    """Mock object detection message."""
    detection = Mock()
    detection.type = "duckie"
    detection.confidence = 0.85
    detection.x = 320
    detection.y = 350
    detection.w = 60
    detection.h = 40
    return detection


@pytest.fixture
def mock_ros_node():
    """Mock ROS node for testing."""
    node = Mock()
    node.get_param = Mock(return_value=None)
    node.set_param = Mock()
    node.publish = Mock()
    node.subscribe = Mock()
    node.loginfo = Mock()
    node.logwarn = Mock()
    node.logerr = Mock()
    return node


# Utility functions for tests
def assert_image_valid(image):
    """Assert that an image is valid for testing."""
    assert image is not None
    assert len(image.shape) == 3
    assert image.shape[2] == 3
    assert image.dtype == np.uint8


def assert_detection_valid(detection):
    """Assert that a detection object is valid."""
    assert hasattr(detection, 'x')
    assert hasattr(detection, 'y')
    assert hasattr(detection, 'w')
    assert hasattr(detection, 'h')
    assert 0 <= detection.x < MOCK_IMAGE_SIZE[0]
    assert 0 <= detection.y < MOCK_IMAGE_SIZE[1]


def create_test_image_with_color(color_bgr, size=(100, 100)):
    """Create a test image filled with a specific color."""
    image = np.full((size[1], size[0], 3), color_bgr, dtype=np.uint8)
    return image 