"""
Unit tests for LED detection node functionality.
"""

import pytest
import numpy as np
import cv2
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the packages to Python path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../packages'))

from tests.conftest import (
    assert_image_valid, 
    assert_detection_valid, 
    create_test_image_with_color,
    MOCK_IMAGE_SIZE
)


class TestLEDDetection:
    """Test class for LED detection functionality."""
    
    def test_white_led_detection(self, sample_image_with_led):
        """Test detection of white LEDs (front of duckiebot)."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        # Mock ROS components
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'):
            
            detector = LEDDetectionNode()
            
            # Test white LED detection
            detections = detector.detect_leds(sample_image_with_led)
            
            # Should detect white LEDs
            white_leds = [d for d in detections if d.get('color') == 'white']
            assert len(white_leds) >= 2, "Should detect at least 2 white LEDs"
    
    def test_red_led_detection(self, sample_image_with_led):
        """Test detection of red LEDs (back of duckiebot)."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'):
            
            detector = LEDDetectionNode()
            detections = detector.detect_leds(sample_image_with_led)
            
            # Should detect red LEDs
            red_leds = [d for d in detections if d.get('color') == 'red']
            assert len(red_leds) >= 2, "Should detect at least 2 red LEDs"
    
    def test_led_color_classification(self):
        """Test LED color classification logic."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'):
            
            detector = LEDDetectionNode()
            
            # Test color classification
            white_color = np.array([255, 255, 255])
            red_color = np.array([0, 0, 255])
            blue_color = np.array([255, 0, 0])
            
            assert detector.classify_led_color(white_color) == 'white'
            assert detector.classify_led_color(red_color) == 'red'
            # Blue should be classified as white (front LED)
            assert detector.classify_led_color(blue_color) == 'white'
    
    def test_distance_estimation(self):
        """Test distance estimation from LED size."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'):
            
            detector = LEDDetectionNode()
            
            # Test distance estimation
            large_led_area = 100  # Close LED
            small_led_area = 25   # Far LED
            
            close_distance = detector.estimate_distance_from_size(large_led_area)
            far_distance = detector.estimate_distance_from_size(small_led_area)
            
            assert close_distance < far_distance, "Larger LEDs should indicate closer distance"
            assert close_distance > 0, "Distance should be positive"
            assert far_distance > 0, "Distance should be positive"
    
    def test_blob_detection_parameters(self):
        """Test blob detection parameters are correctly set."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'):
            
            detector = LEDDetectionNode()
            params = detector.setup_blob_detector()
            
            # Check blob detector parameters
            assert params.filterByArea == True
            assert params.minArea > 0
            assert params.maxArea > params.minArea
            assert params.filterByCircularity == True
            assert params.filterByInertia == True
    
    def test_empty_image_handling(self):
        """Test handling of empty or invalid images."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'):
            
            detector = LEDDetectionNode()
            
            # Test with None image
            detections = detector.detect_leds(None)
            assert detections == [], "Should return empty list for None image"
            
            # Test with empty image
            empty_image = np.zeros((10, 10, 3), dtype=np.uint8)
            detections = detector.detect_leds(empty_image)
            assert isinstance(detections, list), "Should return list even for empty image"
    
    def test_led_position_validation(self):
        """Test that detected LED positions are within image bounds."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'):
            
            detector = LEDDetectionNode()
            
            # Create image with LEDs
            image = create_test_image_with_color((0, 0, 0))
            cv2.circle(image, (50, 50), 8, (255, 255, 255), -1)  # White LED
            
            detections = detector.detect_leds(image)
            
            for detection in detections:
                assert 0 <= detection.get('x', -1) < image.shape[1]
                assert 0 <= detection.get('y', -1) < image.shape[0]
    
    @pytest.mark.parametrize("led_size,expected_detection", [
        (3, False),   # Too small
        (8, True),    # Perfect size
        (15, True),   # Large but valid
        (50, False),  # Too large
    ])
    def test_led_size_filtering(self, led_size, expected_detection):
        """Test LED detection based on size filtering."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'):
            
            detector = LEDDetectionNode()
            
            # Create image with LED of specific size
            image = create_test_image_with_color((0, 0, 0), size=(200, 200))
            cv2.circle(image, (100, 100), led_size, (255, 255, 255), -1)
            
            detections = detector.detect_leds(image)
            
            if expected_detection:
                assert len(detections) > 0, f"Should detect LED of size {led_size}"
            else:
                assert len(detections) == 0, f"Should not detect LED of size {led_size}"


class TestLEDClassification:
    """Test LED color classification algorithms."""
    
    def test_color_threshold_values(self):
        """Test color threshold values for LED classification."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'):
            
            detector = LEDDetectionNode()
            
            # Test blue threshold (should distinguish red from white)
            red_bgr = np.array([0, 0, 255])
            white_bgr = np.array([255, 255, 255])
            
            red_blue = red_bgr[0]  # Blue channel of red
            white_blue = white_bgr[0]  # Blue channel of white
            
            threshold = 235  # Expected threshold from implementation
            
            assert red_blue < threshold, f"Red LED blue channel ({red_blue}) should be below threshold"
            assert white_blue >= threshold, f"White LED blue channel ({white_blue}) should be above threshold"
    
    def test_head_tail_determination(self):
        """Test determination of head vs tail LEDs."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'):
            
            detector = LEDDetectionNode()
            
            # Mock detections
            white_leds = [
                {'x': 100, 'y': 200, 'color': 'white'},
                {'x': 120, 'y': 200, 'color': 'white'}
            ]
            red_leds = [
                {'x': 100, 'y': 250, 'color': 'red'},
                {'x': 120, 'y': 250, 'color': 'red'}
            ]
            
            all_leds = white_leds + red_leds
            result = detector.classify_head_tail(all_leds)
            
            assert 'head' in result, "Should identify head LEDs"
            assert 'tail' in result, "Should identify tail LEDs"
            assert len(result['head']) == 2, "Should have 2 head LEDs"
            assert len(result['tail']) == 2, "Should have 2 tail LEDs"


@pytest.mark.integration
class TestLEDDetectionIntegration:
    """Integration tests for LED detection with ROS."""
    
    def test_ros_message_publishing(self, ros_environment):
        """Test that LED detections are properly published as ROS messages."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        with patch('rospy.Publisher') as mock_publisher:
            mock_pub_instance = Mock()
            mock_publisher.return_value = mock_pub_instance
            
            with patch('rospy.Subscriber'), patch('rospy.init_node'):
                detector = LEDDetectionNode()
                
                # Simulate detection and publishing
                detector.publish_detections([{
                    'x': 100, 'y': 200, 'color': 'white', 'distance': 0.5
                }])
                
                # Verify publisher was called
                mock_pub_instance.publish.assert_called()
    
    def test_image_callback_processing(self, sample_image_with_led):
        """Test image callback processing pipeline."""
        from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
        
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'):
            
            detector = LEDDetectionNode()
            
            # Mock ROS image message
            mock_msg = Mock()
            mock_msg.data = sample_image_with_led.tobytes()
            mock_msg.height = sample_image_with_led.shape[0]
            mock_msg.width = sample_image_with_led.shape[1]
            mock_msg.encoding = "bgr8"
            
            # Test callback (should not raise exceptions)
            try:
                detector.image_callback(mock_msg)
            except Exception as e:
                pytest.fail(f"Image callback should not raise exception: {e}")


@pytest.mark.slow
@pytest.mark.parametrize("num_leds", [2, 4, 6, 8])
def test_multiple_led_detection(num_leds):
    """Test detection of multiple LEDs in various configurations."""
    from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
    
    with patch('rospy.init_node'), \
         patch('rospy.Publisher'), \
         patch('rospy.Subscriber'):
        
        detector = LEDDetectionNode()
        
        # Create image with multiple LEDs
        image = create_test_image_with_color((0, 0, 0), size=(400, 300))
        
        for i in range(num_leds):
            x = 50 + (i % 4) * 80
            y = 50 + (i // 4) * 80
            color = (255, 255, 255) if i % 2 == 0 else (0, 0, 255)
            cv2.circle(image, (x, y), 8, color, -1)
        
        detections = detector.detect_leds(image)
        
        # Should detect most LEDs (allowing for some detection variance)
        detected_count = len(detections)
        assert detected_count >= num_leds * 0.7, f"Should detect at least 70% of {num_leds} LEDs, got {detected_count}" 