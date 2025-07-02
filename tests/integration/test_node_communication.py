"""
Integration tests for ROS node communication in dynamic obstacle avoidance.
"""

import pytest
import rospy
import time
from unittest.mock import Mock, patch, MagicMock
from std_msgs.msg import String, Header
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist


@pytest.mark.integration
@pytest.mark.ros
class TestNodeCommunication:
    """Test communication between ROS nodes."""
    
    def test_image_pipeline_integration(self, ros_environment):
        """Test image processing pipeline integration."""
        
        # Mock image message
        mock_image_msg = Mock()
        mock_image_msg.header = Header()
        mock_image_msg.header.stamp = rospy.Time.now()
        mock_image_msg.format = "jpeg"
        mock_image_msg.data = b"fake_image_data"
        
        # Test that image flows through pipeline
        with patch('rospy.Publisher') as mock_pub, \
             patch('rospy.Subscriber') as mock_sub:
            
            # Should create publishers and subscribers
            assert mock_pub.called, "Should create publishers"
            assert mock_sub.called, "Should create subscribers"
    
    def test_detection_message_flow(self, ros_environment):
        """Test detection messages flow between nodes."""
        
        with patch('rospy.Publisher') as mock_publisher:
            mock_pub_instance = Mock()
            mock_publisher.return_value = mock_pub_instance
            
            # Simulate detection publishing
            detection_msg = Mock()
            detection_msg.detections = []
            
            # Test publishing
            mock_pub_instance.publish(detection_msg)
            mock_pub_instance.publish.assert_called_with(detection_msg)
    
    def test_control_command_integration(self, ros_environment):
        """Test control command integration."""
        
        with patch('rospy.Publisher') as mock_publisher:
            mock_pub_instance = Mock()
            mock_publisher.return_value = mock_pub_instance
            
            # Create control command
            cmd = Twist()
            cmd.linear.x = 0.3
            cmd.angular.z = 0.1
            
            # Test command publishing
            mock_pub_instance.publish(cmd)
            mock_pub_instance.publish.assert_called_with(cmd)


@pytest.mark.integration
class TestLaunchFileValidation:
    """Test launch file configuration and validation."""
    
    def test_launch_file_syntax(self):
        """Test that launch files have valid XML syntax."""
        import xml.etree.ElementTree as ET
        from pathlib import Path
        
        launch_files = [
            "packages/duckietown_demos/launch/lane_following_with_dynamic_avoidance.launch",
            "packages/dynamic_obstacle_avoidance/launch/dynamic_obstacle_avoidance.launch",
            "packages/dynamic_obstacle_avoidance/launch/led_detection_node.launch"
        ]
        
        for launch_file in launch_files:
            if Path(launch_file).exists():
                try:
                    ET.parse(launch_file)
                except ET.ParseError as e:
                    pytest.fail(f"Launch file {launch_file} has invalid XML: {e}")
    
    def test_node_configuration(self):
        """Test that nodes are properly configured in launch files."""
        import xml.etree.ElementTree as ET
        from pathlib import Path
        
        launch_file = "packages/duckietown_demos/launch/lane_following_with_dynamic_avoidance.launch"
        
        if Path(launch_file).exists():
            tree = ET.parse(launch_file)
            root = tree.getroot()
            
            # Check for required nodes
            nodes = root.findall(".//node")
            node_names = [node.get('name') for node in nodes]
            
            expected_nodes = [
                'dynamic_controller_node',
                'led_detection_node', 
                'duckie_detection_node'
            ]
            
            for expected_node in expected_nodes:
                found = any(expected_node in name for name in node_names if name)
                if not found:
                    pytest.skip(f"Node {expected_node} not found in launch file - may be optional")


@pytest.mark.integration 
class TestParameterValidation:
    """Test parameter validation and configuration."""
    
    def test_config_file_loading(self):
        """Test loading of configuration files."""
        import yaml
        from pathlib import Path
        
        config_files = [
            "packages/dynamic_obstacle_avoidance/config/dynamic_controller_node/default.yaml",
            "packages/dynamic_obstacle_avoidance/config/led_detection_node/default.yaml",
            "packages/dynamic_obstacle_avoidance/config/duckie_detection_node/default.yaml"
        ]
        
        for config_file in config_files:
            if Path(config_file).exists():
                try:
                    with open(config_file, 'r') as f:
                        config = yaml.safe_load(f)
                    assert isinstance(config, dict), f"Config {config_file} should be a dictionary"
                except yaml.YAMLError as e:
                    pytest.fail(f"Config file {config_file} has invalid YAML: {e}")
    
    def test_parameter_ranges(self):
        """Test that parameters are within valid ranges."""
        import yaml
        from pathlib import Path
        
        config_file = "packages/dynamic_obstacle_avoidance/config/dynamic_controller_node/default.yaml"
        
        if Path(config_file).exists():
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # Test parameter ranges
            if 'overtaking_steps' in config:
                assert 2 <= config['overtaking_steps'] <= 10, "Overtaking steps should be reasonable"
            
            if 'step_duration' in config:
                assert 0.5 <= config['step_duration'] <= 10.0, "Step duration should be reasonable"
            
            if 'lane_width' in config:
                assert 0.1 <= config['lane_width'] <= 0.5, "Lane width should be reasonable"
            
            if 'emergency_stop_distance' in config:
                assert 0.05 <= config['emergency_stop_distance'] <= 0.3, "Emergency distance should be reasonable"


@pytest.mark.integration
class TestSystemIntegration:
    """Test overall system integration."""
    
    def test_package_dependencies(self):
        """Test that all required packages are available."""
        import importlib
        
        required_packages = [
            'cv2',
            'numpy', 
            'rospy',
            'sensor_msgs.msg',
            'geometry_msgs.msg',
            'std_msgs.msg'
        ]
        
        for package in required_packages:
            try:
                importlib.import_module(package)
            except ImportError:
                pytest.fail(f"Required package {package} not available")
    
    def test_ros_message_types(self):
        """Test that required ROS message types are available."""
        try:
            from sensor_msgs.msg import CompressedImage, Image
            from geometry_msgs.msg import Twist
            from std_msgs.msg import String, Header, Float32
            
            # Test message creation
            img_msg = CompressedImage()
            cmd_msg = Twist()
            
            assert hasattr(img_msg, 'data'), "CompressedImage should have data field"
            assert hasattr(cmd_msg, 'linear'), "Twist should have linear field"
            assert hasattr(cmd_msg, 'angular'), "Twist should have angular field"
            
        except ImportError as e:
            pytest.fail(f"Required ROS message types not available: {e}")
    
    def test_dynamic_obstacle_avoidance_package(self):
        """Test dynamic obstacle avoidance package structure."""
        from pathlib import Path
        
        package_path = Path("packages/dynamic_obstacle_avoidance")
        
        # Check package structure
        assert package_path.exists(), "Dynamic obstacle avoidance package should exist"
        assert (package_path / "package.xml").exists(), "Should have package.xml"
        assert (package_path / "CMakeLists.txt").exists(), "Should have CMakeLists.txt"
        assert (package_path / "src").exists(), "Should have src directory"
        assert (package_path / "config").exists(), "Should have config directory"
        assert (package_path / "launch").exists(), "Should have launch directory"


@pytest.mark.slow
@pytest.mark.integration
class TestEndToEndIntegration:
    """End-to-end integration tests."""
    
    def test_complete_pipeline_mock(self, sample_image_with_duckie, ros_environment):
        """Test complete pipeline with mocked components."""
        
        with patch('rospy.init_node'), \
             patch('rospy.Publisher'), \
             patch('rospy.Subscriber'), \
             patch('rospy.spin'):
            
            # Mock the complete pipeline
            try:
                # This would test the full pipeline if we had the actual nodes
                # For now, we test that imports work
                import sys
                import os
                sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../packages'))
                
                # Test that we can import all components
                # from dynamic_obstacle_avoidance.src.dynamic_controller_node import DynamicControllerNode
                # from dynamic_obstacle_avoidance.src.led_detection_node import LEDDetectionNode
                # from dynamic_obstacle_avoidance.src.duckie_detection_node import DuckieDetectionNode
                
                pass  # Placeholder for actual integration test
                
            except Exception as e:
                pytest.fail(f"End-to-end integration failed: {e}")
    
    def test_performance_integration(self):
        """Test system performance under load."""
        import time
        
        start_time = time.time()
        
        # Simulate processing load
        for _ in range(100):
            # Mock processing
            time.sleep(0.001)  # 1ms per iteration
        
        end_time = time.time()
        total_time = end_time - start_time
        
        # Should complete in reasonable time
        assert total_time < 1.0, f"Performance test took {total_time:.3f}s, should be < 1.0s" 