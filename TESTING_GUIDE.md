# 🧪 Testing Guide for Duckietown Dynamic Obstacle Avoidance

This guide provides comprehensive information about testing the integrated Duckietown lane following system with dynamic obstacle avoidance.

## 📋 Test Overview

The testing suite is organized into multiple layers:

- **Unit Tests**: Test individual components in isolation
- **Integration Tests**: Test component interactions and ROS communication
- **Simulation Tests**: Test complete scenarios with synthetic data
- **Safety Tests**: Critical safety-related functionality
- **Performance Tests**: Performance benchmarks and load testing

## 🚀 Quick Start

### Install Test Dependencies
```bash
pip install -r requirements-test.txt
```

### Run All Tests
```bash
python run_tests.py --all --verbose
```

### Run Specific Test Categories
```bash
# Unit tests only
python run_tests.py --unit

# Integration tests
python run_tests.py --integration

# Safety-critical tests
python run_tests.py --safety

# Performance tests
python run_tests.py --performance
```

## 📁 Test Structure

```
tests/
├── __init__.py                 # Test package init
├── conftest.py                 # Shared fixtures and utilities
├── unit/                       # Unit tests
│   ├── test_led_detection.py   # LED detection tests
│   ├── test_duckie_detection.py # Duckie detection tests
│   └── test_dynamic_controller.py # Controller logic tests
├── integration/                # Integration tests
│   └── test_node_communication.py # ROS node integration
├── simulation/                 # Simulation tests
│   └── test_scenarios.py       # Scenario-based tests
└── data/                       # Test data and fixtures
```

## 🔧 Test Configuration

### Pytest Configuration (`pytest.ini`)
```ini
[tool:pytest]
testpaths = tests
markers =
    unit: Unit tests for individual components
    integration: Integration tests for node communication
    simulation: Simulation tests with mock data
    safety: Critical safety-related tests
    slow: Tests that take a long time to run
```

### Running Tests with Markers
```bash
# Run only unit tests
pytest -m unit

# Run only safety-critical tests
pytest -m safety

# Run everything except slow tests
pytest -m "not slow"

# Run integration and simulation tests
pytest -m "integration or simulation"
```

## 🧪 Test Categories

### Unit Tests

**LED Detection Tests (`test_led_detection.py`)**
- White/red LED color classification
- Distance estimation from LED size
- Blob detection parameter validation
- Position validation and size filtering
- Head/tail LED determination

**Duckie Detection Tests (`test_duckie_detection.py`)**
- HSV color range validation for yellow detection
- Color mask creation and filtering
- Lane position analysis
- Distance estimation from duckie size
- Tracking and outlier filtering

**Dynamic Controller Tests (`test_dynamic_controller.py`)**
- State machine transitions (NORMAL → OVERTAKING → EMERGENCY_STOP)
- Decision making based on obstacle distance and position
- Lane offset calculation for sinusoidal overtaking
- Safety checks for left lane clearance
- Emergency stop command generation

### Integration Tests

**Node Communication Tests (`test_node_communication.py`)**
- ROS message publishing and subscribing
- Image pipeline integration
- Control command flow
- Launch file validation
- Parameter loading and validation

### Simulation Tests

**Scenario Tests (`test_scenarios.py`)**
- Static duckie overtaking scenarios
- Static duckiebot detection and avoidance
- Moving obstacle scenarios with relative velocity
- Emergency stop scenarios
- Multi-obstacle decision making
- Different lighting conditions

## 📊 Coverage and Reporting

### Generate Coverage Report
```bash
python run_tests.py --all --coverage
```

This generates:
- Terminal coverage summary
- HTML coverage report in `htmlcov/index.html`

### Generate Test Report
```bash
python run_tests.py --report
```

This generates:
- HTML test report: `test_report.html`
- JUnit XML: `test_results.xml`
- Coverage report: `htmlcov/`

## 🔍 Test Fixtures and Utilities

### Common Fixtures (`conftest.py`)

**Image Fixtures:**
- `sample_image`: Basic lane image with white/yellow lines
- `sample_image_with_duckie`: Image containing yellow duckie
- `sample_image_with_led`: Image with LED patterns

**Mock Objects:**
- `mock_lane_pose`: Mock lane pose message
- `mock_car_cmd`: Mock car command message
- `mock_fsm_state`: Mock FSM state message
- `mock_ros_node`: Mock ROS node

**Utilities:**
- `assert_image_valid()`: Validate image properties
- `assert_detection_valid()`: Validate detection objects
- `create_test_image_with_color()`: Generate test images

## 🛡️ Safety Testing

Safety tests are marked with `@pytest.mark.safety` and include:

- Emergency stop functionality in all states
- Collision avoidance logic
- State machine loop prevention
- Critical distance thresholds

**Run safety tests:**
```bash
python run_tests.py --safety
```

## ⚡ Performance Testing

Performance tests validate:
- Detection processing speed
- Memory usage under load
- Frame rate maintenance
- Continuous operation stability

**Run performance tests:**
```bash
python run_tests.py --performance
```

## 🐛 Debugging Tests

### Run Tests with Debugging
```bash
# Run with pdb on failure
pytest --pdb

# Run specific test with verbose output
pytest tests/unit/test_led_detection.py::TestLEDDetection::test_white_led_detection -v

# Run with custom markers
pytest -m "unit and not slow" -v
```

### Test Output and Logging
```bash
# Show print statements
pytest -s

# Show log output
pytest --log-cli-level=INFO

# Capture stdout/stderr
pytest --capture=no
```

## 📝 Writing New Tests

### Test Naming Convention
- Test files: `test_*.py`
- Test classes: `Test*`
- Test functions: `test_*`

### Example Unit Test
```python
import pytest
from unittest.mock import Mock, patch

class TestMyComponent:
    def test_basic_functionality(self, mock_ros_node):
        """Test basic functionality of component."""
        # Arrange
        component = MyComponent()
        
        # Act
        result = component.process_data(test_input)
        
        # Assert
        assert result.is_valid()
        assert result.value > 0
    
    @pytest.mark.parametrize("input,expected", [
        (1, 2),
        (2, 4),
        (3, 6)
    ])
    def test_parameterized(self, input, expected):
        """Test with multiple parameter sets."""
        assert process(input) == expected
```

### Integration Test Example
```python
@pytest.mark.integration
@pytest.mark.ros
class TestNodeIntegration:
    def test_message_flow(self, ros_environment):
        """Test message flow between nodes."""
        with patch('rospy.Publisher') as mock_pub:
            # Test node communication
            pass
```

## 🔄 Continuous Integration

### Pre-commit Checks
```bash
# Run all quality checks
python run_tests.py --lint --format --unit --safety
```

### CI Pipeline Commands
```bash
# Full test suite for CI
python run_tests.py --all --coverage --report

# Quick validation
python run_tests.py --unit --safety --lint
```

## 📈 Test Metrics

### Coverage Targets
- **Unit Tests**: > 90% line coverage
- **Integration Tests**: > 80% component coverage
- **Safety Tests**: 100% critical path coverage

### Performance Targets
- **Detection Speed**: < 100ms per frame
- **Memory Usage**: < 500MB continuous operation
- **Frame Rate**: > 10 FPS under load

## 🛠️ Troubleshooting

### Common Issues

**ImportError for packages:**
```bash
# Add packages to Python path
export PYTHONPATH="${PYTHONPATH}:$(pwd)/packages"
```

**ROS not available:**
```bash
# Skip ROS tests
pytest -m "not ros"
```

**Slow tests timing out:**
```bash
# Skip slow tests
pytest -m "not slow"

# Increase timeout
pytest --timeout=300
```

### Mock ROS Environment
For testing without full ROS setup:
```python
@patch('rospy.init_node')
@patch('rospy.Publisher')
@patch('rospy.Subscriber')
def test_without_ros():
    # Test logic here
    pass
```

## 📚 Additional Resources

- [pytest Documentation](https://docs.pytest.org/)
- [unittest.mock](https://docs.python.org/3/library/unittest.mock.html)
- [ROS Testing](http://wiki.ros.org/rostest)
- [OpenCV Testing](https://docs.opencv.org/master/d7/d4d/tutorial_py_tests.html)

---

**Happy Testing! 🦆🚗**

Remember: Good tests are the foundation of reliable autonomous systems! 