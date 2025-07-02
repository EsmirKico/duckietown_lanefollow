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
│   ├── dynamic_obstacle_avoidance/  ← NEW: Smart obstacle avoidance
│   ├── object_detection/           ← YOLOv5 AI detection
│   ├── lane_control/              ← PID lane following
│   └── duckietown_demos/          ← Launch files
└── DEPLOYMENT_GUIDE.md           ← Complete instructions

proj-lfvop-master/              ← REFERENCE ONLY (can be deleted)
├── Dockerfile                  ← Don't use this
├── dependencies-*.txt          ← Already merged into main project
└── packages/                   ← Already integrated above
```

## 🎯 What's Integrated

| Feature | Status | Location |
|---------|--------|----------|
| Lane Following | ✅ | `packages/lane_control/` |
| Object Detection (YOLOv5) | ✅ | `packages/object_detection/` |
| Dynamic Obstacle Avoidance | ✅ | `packages/dynamic_obstacle_avoidance/` |
| LED Detection | ✅ | `packages/dynamic_obstacle_avoidance/` |
| HSV Duckie Detection | ✅ | `packages/dynamic_obstacle_avoidance/` |
| Smart Overtaking Logic | ✅ | `packages/dynamic_obstacle_avoidance/` |

## 🗑 Cleanup Instructions

**You can safely delete the reference folder:**

```bash
cd /path/to/your/workspace
rm -rf proj-lfvop-master/
```

The `proj-lfvop-master` folder was only used as a source for extracting and integrating the dynamic obstacle avoidance features. All functionality has been merged into the main `duckietown_lanefollow` project.

## 🔄 Behavior Comparison

| Situation | Old Behavior | New Behavior |
|-----------|--------------|--------------|
| Duckie on lane | Stop and wait | Smooth overtaking |
| Another Duckiebot | Stop and wait | Dynamic lane switching |
| Moving obstacle | Stop and wait | Intelligent avoidance |
| Emergency case | Stop | Emergency stop (safety) |

## 🛠 Available Launch Configurations

```bash
# Basic lane following only
roslaunch duckietown_demos lane_following.launch veh:=[ROBOT_NAME]

# Enhanced with dynamic avoidance (RECOMMENDED)
roslaunch duckietown_demos lane_following_with_dynamic_avoidance.launch veh:=[ROBOT_NAME]

# Jetson optimized version
roslaunch duckietown_demos lane_following_jetson_optimized.launch veh:=[ROBOT_NAME]
```

## 📖 Documentation

- **[DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md)** - Complete setup and usage instructions
- **[packages/dynamic_obstacle_avoidance/README.md](packages/dynamic_obstacle_avoidance/README.md)** - Technical details of avoidance system

## 🤝 Integration Benefits

✅ **Single Dockerfile** - No confusion about which image to build  
✅ **Unified Dependencies** - All requirements in one place  
✅ **One Launch Command** - Simple deployment process  
✅ **Backward Compatible** - Can still run basic lane following  
✅ **No Duplicate Code** - Clean integration without redundancy  

## 🚨 Important Notes

1. **Use only the main `Dockerfile`** in the root directory
2. **The `proj-lfvop-master` folder is reference only** and can be deleted
3. **All features are accessible through launch files** in `packages/duckietown_demos/launch/`
4. **Dependencies are automatically handled** during the Docker build

---

**Ready to see your Duckiebot intelligently navigate around obstacles! 🦆🚗**
