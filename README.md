# Duckietown Lane Following with Dynamic Obstacle Avoidance

A unified autonomous navigation system for Duckiebots that combines intelligent lane following with dynamic obstacle avoidance capabilities.

## 🚀 Quick Start

**This is the main project. Use only this folder and its Dockerfile.**

```bash
# 1. Build the integrated system
dts devel build -f --arch arm32v7 -H [DUCKIEBOT_NAME].local

# 2. Run with dynamic obstacle avoidance
docker -H [DUCKIEBOT_NAME].local run -it --rm --net host -v /data/:/data/ \
  duckietown/dt-core:master-arm32v7 \
  roslaunch duckietown_demos lane_following_with_dynamic_avoidance.launch veh:=[DUCKIEBOT_NAME]
```

## 📁 Project Structure

```
duckietown_lanefollow/          ← USE THIS (Main integrated project)
├── Dockerfile                  ← Single build file for everything
├── dependencies-py3.txt        ← All dependencies in one place
├── packages/
│   ├── dynamic_obstacle_avoidance/  ← NEW: Smart overtaking logic
│   ├── object_detection/           ← YOLOv5 detection
│   ├── lane_control/              ← PID lane following
│   └── ...                        ← Other Duckietown packages
├── launchers/
│   └── default.sh                 ← Main launcher
└── DEPLOYMENT_GUIDE.md           ← Complete deployment instructions
```

## 🎯 Features

### Core Capabilities
- **Lane Following**: Advanced PID control with line detection
- **Object Detection**: YOLOv5-based detection (duckies, cones, trucks, buses)
- **Dynamic Obstacle Avoidance**: Smart overtaking maneuvers instead of just stopping
- **Multi-Modal Detection**: AI + Computer Vision (LED detection + HSV color detection)
- **Safety Systems**: Emergency stops and collision avoidance

### Key Improvements Over Standard Lane Following
- ✅ **Dynamic Avoidance**: Goes AROUND obstacles instead of just stopping
- ✅ **Smart Decision Making**: Analyzes left lane before overtaking
- ✅ **Smooth Maneuvers**: 4-step sinusoidal lane transitions
- ✅ **Multi-Modal Detection**: YOLOv5 + LED detection + HSV filtering
- ✅ **Safety First**: Emergency stops for dangerous situations

## 🧪 Testing

Comprehensive testing suite with unit, integration, simulation, and safety tests.

### Install Test Dependencies
```bash
pip install -r requirements-test.txt
```

### Run Tests
```bash
# Quick test
python run_tests.py --unit --safety

# Full test suite with coverage
python run_tests.py --all --coverage

# Specific test categories
python run_tests.py --unit          # Unit tests only
python run_tests.py --integration   # Integration tests
python run_tests.py --simulation    # Scenario tests
python run_tests.py --safety        # Safety-critical tests
```

See [`TESTING_GUIDE.md`](TESTING_GUIDE.md) for detailed testing information.

## 📋 Prerequisites

- Duckiebot in configuration `DB18` using `daffy` version
- Camera calibration completed
- Duckietown setup with white and yellow lanes
- At least one Duckiebot (multiple for testing overtaking)

## 🚀 Deployment

See [`DEPLOYMENT_GUIDE.md`](DEPLOYMENT_GUIDE.md) for complete deployment instructions.

### Quick Deploy
```bash
# Build
dts devel build -f --arch arm32v7 -H [DUCKIEBOT_NAME].local

# Run lane following with dynamic avoidance
dts duckiebot demo --demo_name lane_following_with_dynamic_avoidance --duckiebot_name [DUCKIEBOT_NAME] --package_name duckietown_demos
```

## 🔧 Configuration

Key parameters in `packages/dynamic_obstacle_avoidance/config/`:

- **Overtaking**: 4 steps, 4-second transitions, 0.22m lane width
- **Safety**: Emergency stop at 0.15m distance
- **Detection**: YOLOv5 confidence > 0.7, LED blob detection
- **Control**: Sinusoidal lane offset with PID integration

## 🛡️ Safety Features

- **Emergency Stop**: Immediate stop for obstacles < 15cm
- **Left Lane Check**: Verify left lane is clear before overtaking
- **Collision Avoidance**: Multi-modal detection prevents false negatives
- **State Machine**: Robust transitions prevent deadlocks
- **Fail-Safe**: Returns to normal lane following if avoidance fails

## 📊 Performance

- **Detection Speed**: < 100ms per frame
- **Overtaking Time**: 16 seconds (4 steps × 4 seconds)
- **Safety Distance**: 15cm emergency threshold
- **Detection Range**: 1+ meter obstacle detection

## 🤝 Integration

### Existing Components Used
- `lane_control`: PID controller for lane following
- `object_detection`: YOLOv5 for duckie/vehicle detection
- `image_processing`: Camera feed and image rectification
- `ground_projection`: Lane pose estimation

### New Components Added
- `dynamic_obstacle_avoidance`: Complete avoidance system
  - `dynamic_controller_node`: Decision making and control
  - `led_detection_node`: Duckiebot LED detection
  - `duckie_detection_node`: HSV-based duckie detection

## 🐛 Troubleshooting

### Common Issues

**No obstacles detected:**
- Check camera calibration
- Verify lighting conditions
- Confirm YOLOv5 model is loaded

**Erratic overtaking:**
- Check lane pose calibration
- Verify `d_offset` parameters
- Ensure left lane detection is working

**Emergency stops:**
- Check emergency distance threshold (0.15m)
- Verify obstacle distance estimation
- Confirm safety logic parameters

### Debug Commands
```bash
# Check detection pipeline
rostopic echo /[DUCKIEBOT_NAME]/led_detection_node/detections

# Monitor control commands
rostopic echo /[DUCKIEBOT_NAME]/car_cmd

# View processed images
rqt_image_view /[DUCKIEBOT_NAME]/camera_node/image/compressed
```

## 📚 Documentation

- [`DEPLOYMENT_GUIDE.md`](DEPLOYMENT_GUIDE.md) - Complete deployment instructions
- [`TESTING_GUIDE.md`](TESTING_GUIDE.md) - Testing framework and guidelines
- [`CLEANUP_GUIDE.md`](CLEANUP_GUIDE.md) - Project cleanup instructions

## 🏆 Results

This integrated system transforms a basic lane-following robot into an intelligent autonomous vehicle capable of:

1. **Detecting** obstacles using multiple methods (AI + computer vision)
2. **Analyzing** the situation (distance, lane availability, safety)
3. **Deciding** on the best action (overtake, wait, emergency stop)
4. **Executing** smooth maneuvers (sinusoidal lane transitions)
5. **Ensuring safety** (emergency stops, collision avoidance)

**From "stop-and-wait" to "detect-analyze-overtake" autonomous navigation!** 🦆🚗

---

## 📜 License

This project builds upon the Duckietown ecosystem and follows the same open-source principles.
