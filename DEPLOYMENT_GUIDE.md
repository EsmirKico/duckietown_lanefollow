# Duckietown Lane Following with Dynamic Obstacle Avoidance

**A unified Duckietown autonomous navigation system combining lane following, object detection, and dynamic obstacle avoidance.**

## 🚀 Key Features

- **Lane Following**: Advanced PID control with line detection
- **Object Detection**: YOLOv5-based detection (duckies, cones, trucks, buses)
- **Dynamic Obstacle Avoidance**: Smart overtaking maneuvers instead of just stopping
- **Multi-Modal Detection**: AI + Computer Vision (LED detection + HSV color detection)
- **Safety Systems**: Emergency stops and collision avoidance

## 📋 Prerequisites

- Duckiebot in configuration `DB18` using `daffy` version
- Camera calibration completed
- Duckietown setup with white and yellow lanes
- At least one Duckiebot (multiple for dynamic testing)

## 🛠 Building the System

### 1. Clone and Build
```bash
git clone <this-repository>
cd duckietown_lanefollow

# Stop watchtower
dts devel watchtower stop -H [DUCKIEBOT_NAME].local

# Build the integrated image
dts devel build -f --arch arm32v7 -H [DUCKIEBOT_NAME].local
```

### 2. Start Required Containers

**Duckiebot Interface:**
```bash
dts duckiebot demo --demo_name all_drivers --duckiebot_name [DUCKIEBOT_NAME] --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy
```

**Car Interface:**
```bash
dts duckiebot demo --demo_name all --duckiebot_name [DUCKIEBOT_NAME] --package_name car_interface --image duckietown/dt-car-interface:daffy
```

## 🎮 Running Demos

### Demo 1: Basic Lane Following
```bash
docker -H [DUCKIEBOT_NAME].local run -it --rm --net host -v /data/:/data/ duckietown/dt-core:master-arm32v7 roslaunch duckietown_demos lane_following.launch veh:=[DUCKIEBOT_NAME]
```

### Demo 2: Lane Following with Dynamic Obstacle Avoidance
```bash
docker -H [DUCKIEBOT_NAME].local run -it --rm --net host -v /data/:/data/ duckietown/dt-core:master-arm32v7 roslaunch duckietown_demos lane_following_with_dynamic_avoidance.launch veh:=[DUCKIEBOT_NAME]
```

### Demo 3: Keyboard Control
In a separate terminal:
```bash
dts duckiebot keyboard_control [DUCKIEBOT_NAME] --base_image duckietown/dt-core:daffy-amd64
```
Press `a` to start lane following, `s` to stop.

## 🧪 Testing Scenarios

### Static Duckie Avoidance
1. Place a yellow duckie on the lane
2. Start Demo 2 above
3. Use keyboard control to activate lane following
4. Watch the robot smoothly overtake the duckie

### Dynamic Duckiebot Avoidance
1. Set up two Duckiebots
2. Start Demo 2 on both robots
3. Place one robot as an obstacle
4. Control the other robot to see dynamic overtaking

## 🏗 System Architecture

```
Camera → Object Detection (YOLOv5) ↘
      → LED Detection            → Dynamic Controller → Car Commands
      → Duckie Detection         ↗
      → Lane Filter → Lane Controller ↗
```

### Key Components:
- **`object_detection_node`**: YOLOv5-based AI detection
- **`led_detection_node`**: Computer vision LED detection
- **`duckie_detection_node`**: HSV-based duckie detection
- **`dynamic_controller_node`**: Decision making and lane switching
- **`lane_controller_node`**: Basic PID lane following

## 🔧 Configuration

Main configuration files:
- `packages/dynamic_obstacle_avoidance/config/*/default.yaml`
- `packages/lane_control/config/lane_controller_node/default.yaml`
- `packages/object_detection/config/object_detection_node/default.yaml`

## 📁 Project Structure

```
duckietown_lanefollow/
├── Dockerfile                    # Single integrated build file
├── dependencies-py3.txt          # All Python dependencies
├── packages/
│   ├── dynamic_obstacle_avoidance/  # NEW: Dynamic avoidance system
│   ├── object_detection/           # YOLOv5 object detection
│   ├── lane_control/              # PID lane following
│   ├── lane_filter/               # Lane pose estimation
│   └── duckietown_demos/          # Launch files
└── launchers/
    └── default.sh                 # Entry point
```

## 🚨 Important Notes

1. **Single Project**: This is now a unified system. The `proj-lfvop-master` folder is only for reference and can be deleted.

2. **One Dockerfile**: Use only the main `Dockerfile` in the root directory.

3. **Behavior Change**: Unlike basic lane following that stops for obstacles, this system intelligently overtakes them.

4. **Safety First**: The system includes emergency stops for situations too dangerous to overtake.

## 🗑 Cleanup (Optional)

You can safely remove the reference project:
```bash
rm -rf proj-lfvop-master/
```

This folder was only used as a source for integrating dynamic obstacle avoidance features into the main project.

## 🐛 Troubleshooting

- **Build Issues**: Ensure Docker and dts are up to date
- **Performance Issues**: Check camera calibration and lighting conditions
- **Detection Issues**: Verify proper duckie/LED placement and colors
- **Connection Issues**: Ensure all required containers are running

## 📝 Development

To modify behavior:
1. Edit configuration files in `packages/*/config/`
2. Rebuild with `dts devel build`
3. Test with the appropriate demo

---

**This integrated system transforms your Duckiebot from basic lane following to intelligent autonomous navigation with dynamic obstacle avoidance! 🦆🚗** 