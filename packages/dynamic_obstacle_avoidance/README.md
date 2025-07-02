# Dynamic Obstacle Avoidance Package

This package implements dynamic obstacle avoidance for Duckiebots using lane-following with visual obstacle perception (LFVOP). It extends the basic lane following behavior to include intelligent overtaking maneuvers around static and dynamic obstacles.

## Features

### 🤖 **Duckiebot Detection via LED Patterns**
- **White LEDs (Front)**: Detects oncoming Duckiebots in the left lane
- **Red LEDs (Back)**: Detects Duckiebots ahead in the right lane
- Uses HSV color space analysis to distinguish between LED colors
- Provides distance estimation based on LED blob size

### 🦆 **Duckie (Static Obstacle) Detection**
- **Yellow Color Detection**: Uses HSV color space to detect yellow duckies
- **Lane Assignment**: Determines if duckies are in left or right lanes
- **Position Tracking**: Tracks duckie positions with outlier filtering
- **Blob Analysis**: Uses circular blob detection for reliable identification

### 🚗 **Dynamic Lane Switching**
- **Smooth Overtaking**: Uses sinusoidal d_offset parameter manipulation for smooth lane changes
- **Emergency Stopping**: Immediate stop when obstacles are too close (<15cm)
- **Speed Adaptation**: Increases speed during overtaking (configurable)
- **Safety Checks**: Continuous monitoring of left lane before and during overtaking

## Architecture

### Nodes

1. **`led_detection_node.py`**
   - Subscribes to: `camera_node/image/compressed`
   - Publishes: LED detection states and positions
   - Detects white (front) and red (back) LEDs of other Duckiebots

2. **`duckie_detection_node.py`**
   - Subscribes to: `camera_node/image/compressed`, `lane_filter_node/lane_pose`
   - Publishes: Duckie detection states and positions
   - Detects yellow duckies and determines their lane positions

3. **`dynamic_controller_node.py`**
   - Subscribes to: LED detection, duckie detection, lane controller commands
   - Publishes: Modified car commands with obstacle avoidance
   - Implements the main decision logic for stopping vs. overtaking

### Message Types

- **`dynamic_obstacle.msg`**: Custom message containing obstacle states and positions
  ```
  Header header
  uint16[] state    # 0=none, 1=right lane, 2=left lane
  float64[] pos     # [x1, y1, x2, y2, ...] positions in robot frame
  ```

## Configuration

### Parameters (in config files)

**LED Detection Node:**
- `publish_freq`: Detection frequency (default: 2.0 Hz)
- `blue_threshold`: Threshold for distinguishing red vs white LEDs (default: 235)

**Duckie Detection Node:**
- `lane_width`: Half lane width for lane assignment (default: 0.1145m)
- `crop_factor`: Image crop percentage from top (default: 0.3)
- `yellow_low/high`: HSV color range for yellow detection

**Dynamic Controller Node:**
- `nr_steps`: Steps for smooth lane transition (default: 4)
- `transition_time`: Time for complete lane change (default: 4.0s)
- `lanewidth`: Full lane width for overtaking (default: 0.22m)
- `leftlane_time`: Time spent in left lane during overtaking (default: 3s)

## Deployment Steps for DB21J Duckiebot

### 📋 **Prerequisites**
- DB21J Duckiebot with NVIDIA Jetson Nano 4GB
- ROS environment properly sourced (`source /opt/ros/noetic/setup.bash`)
- Catkin workspace set up (`cd ~/catkin_ws`)
- `VEHICLE_NAME` environment variable set (`export VEHICLE_NAME=your_robot_name`)

### 🚀 **New Deployment Workflow (Replaces `dts devel build/run`)**

#### **Step 1: Build the Enhanced System**
```bash
# Navigate to your catkin workspace
cd ~/catkin_ws

# Build the dynamic obstacle avoidance package
catkin_make --pkg dynamic_obstacle_avoidance

# Source the workspace
source devel/setup.bash
```

#### **Step 2: Setup DB21J Optimization (One-time setup)**
```bash
# Make deployment script executable
chmod +x src/duckietown_lanefollow/packages/dynamic_obstacle_avoidance/scripts/deploy_db21j.sh

# Run complete setup and hardware optimization for DB21J
./src/duckietown_lanefollow/packages/dynamic_obstacle_avoidance/scripts/deploy_db21j.sh setup
```

#### **Step 3: Launch Enhanced System**

**🎯 Recommended: Jetson-Optimized Launch**
```bash
# Single command launch with all optimizations
./src/duckietown_lanefollow/packages/dynamic_obstacle_avoidance/scripts/deploy_db21j.sh launch jetson
```

**Alternative Launch Options:**
```bash
# Manual Jetson-optimized launch
roslaunch duckietown_demos lane_following_jetson_optimized.launch veh:=$VEHICLE_NAME

# Standard launch (no Jetson optimizations)
roslaunch duckietown_demos lane_following_with_dynamic_avoidance.launch veh:=$VEHICLE_NAME

# Test only dynamic obstacle avoidance components
./src/duckietown_lanefollow/packages/dynamic_obstacle_avoidance/scripts/deploy_db21j.sh launch test
```

### 🔧 **Development & Testing Commands**

```bash
# Test all system components
./src/duckietown_lanefollow/packages/dynamic_obstacle_avoidance/scripts/deploy_db21j.sh test

# Check system status and performance
./src/duckietown_lanefollow/packages/dynamic_obstacle_avoidance/scripts/deploy_db21j.sh info

# Get help with deployment script
./src/duckietown_lanefollow/packages/dynamic_obstacle_avoidance/scripts/deploy_db21j.sh help
```

### 🎯 **Workflow Comparison**

| **Previous Workflow** | **New Enhanced Workflow** |
|----------------------|---------------------------|
| `dts devel build` | `catkin_make --pkg dynamic_obstacle_avoidance` |
| `dts devel run` | `./deploy_db21j.sh launch jetson` |
| ❌ Only stops at obstacles | ✅ Smoothly navigates around obstacles |
| ❌ No hardware optimization | ✅ Jetson Nano 4GB optimized |
| ❌ Standard performance | ✅ CUDA acceleration when available |
| ❌ Limited obstacle types | ✅ Detects duckies + other Duckiebots |

### 📊 **Performance Monitoring**

```bash
# Real-time performance monitoring
rostopic echo /$VEHICLE_NAME/led_detection_node/detections
rostopic echo /$VEHICLE_NAME/duckie_detection_node/detections  
rostopic echo /$VEHICLE_NAME/dynamic_controller_node/car_cmd

# System health monitoring
watch -n 1 'echo "Temperature: $(cat /sys/devices/virtual/thermal/thermal_zone0/temp | awk "{print \$1/1000}")°C"'
htop  # Monitor CPU/Memory usage
```

## Usage

### Launch Options

1. **🚀 Full System with Dynamic Avoidance (Jetson Optimized):**
   ```bash
   ./src/duckietown_lanefollow/packages/dynamic_obstacle_avoidance/scripts/deploy_db21j.sh launch jetson
   ```

2. **Standard Full System:**
   ```bash
   roslaunch duckietown_demos lane_following_with_dynamic_avoidance.launch veh:=$VEHICLE_NAME
   ```

3. **Standalone Dynamic Avoidance (add to existing system):**
   ```bash
   roslaunch dynamic_obstacle_avoidance dynamic_obstacle_avoidance.launch veh:=$VEHICLE_NAME
   ```

4. **Individual Components:**
   ```bash
   # LED detection only
   roslaunch dynamic_obstacle_avoidance led_detection_node.launch veh:=$VEHICLE_NAME
   ```

### Integration with Existing System

The dynamic controller integrates seamlessly with the existing lane following stack:
- Takes input from `lane_controller_node/car_cmd`
- Outputs to `car_cmd_switch_node/cmd`
- Respects FSM states (only active during "LANE_FOLLOWING")

## Behavior Logic

### Decision Making Process

1. **Obstacle Detection**: Continuously monitors for obstacles in right lane (Duckiebots or duckies)
2. **Safety Check**: Verifies left lane is clear before initiating overtaking
3. **Distance Evaluation**: Only overtakes if obstacle is within range (0.15-0.7m by default)
4. **Emergency Stop**: Immediately stops if obstacle is too close (<0.15m)

### Overtaking Sequence

1. **Go Left**: Smooth transition to left lane using sinusoidal d_offset
2. **Speed Up**: Optional speed increase during overtaking
3. **Monitor Safety**: Continuous left lane monitoring for oncoming traffic
4. **Return Right**: Smooth transition back to right lane
5. **Resume Normal**: Return to standard lane following

## Safety Features

- **Emergency Stopping**: Immediate halt when obstacles are too close
- **Left Lane Monitoring**: Continuous safety checks during overtaking
- **Dynamic Speed Control**: Adjustable speed during maneuvers
- **Outlier Filtering**: Tracks objects over time to prevent false positives
- **Distance-based Decisions**: Different behavior for different obstacle distances

## Dependencies

- Standard Duckietown ROS packages
- OpenCV for computer vision processing
- NumPy for mathematical computations
- No additional external dependencies required

## LED Pattern Setup

For proper detection, ensure your Duckiebot uses the standard LED pattern:
- **Front**: Two white LEDs
- **Back**: Two red LEDs
- **Pattern**: `["white","red","switchedoff","red","white"]`

This pattern is automatically set by the dynamic controller when it starts.

## Troubleshooting

### Common Issues

1. **No LED Detection**: Check camera feed and lighting conditions
2. **False Duckie Detection**: Adjust HSV color thresholds in config
3. **Erratic Overtaking**: Check d_offset parameter integration with lane controller
4. **No Response**: Verify FSM state is "LANE_FOLLOWING"

### Debug Topics

Monitor these topics for debugging:
- `/[robot]/led_detection_node/circlepattern_image`: LED detection visualization
- `/[robot]/duckie_detection_node/duckiedetected_image`: Duckie detection visualization
- `/[robot]/duckie_detection_node/detected_duckie`: Raw duckie detection data
- `/[robot]/dynamic_controller_node/car_cmd`: Final command output

## Performance Notes

- **Processing Frequency**: Runs at 2Hz by default to balance performance and responsiveness
- **CPU Usage**: Moderate due to computer vision processing
- **Memory**: Minimal additional memory footprint
- **Latency**: ~500ms typical response time for obstacle detection to action

This system provides robust dynamic obstacle avoidance while maintaining the safety and reliability of the original lane following behavior. 