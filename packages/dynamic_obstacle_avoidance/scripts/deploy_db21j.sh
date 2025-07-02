#!/bin/bash

# DB21J Dynamic Obstacle Avoidance Deployment Script
# Specifically designed for NVIDIA Jetson Nano 4GB on DB21J Duckiebot
# 
# This script handles:
# - Hardware detection and optimization
# - CUDA setup verification  
# - Performance tuning
# - System testing
# - Launch options

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print functions
print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check if running on Jetson Nano
check_jetson_hardware() {
    print_info "Checking hardware compatibility..."
    
    if [ -f /proc/device-tree/model ]; then
        MODEL=$(cat /proc/device-tree/model)
        if [[ $MODEL == *"jetson"* ]]; then
            print_success "Detected Jetson hardware: $MODEL"
            
            # Check if it's Jetson Nano 4GB (DB21J)
            if [[ $MODEL == *"Nano"* ]]; then
                MEM_GB=$(free -g | awk 'NR==2{printf "%.0f", $2}')
                if [ "$MEM_GB" -ge 3 ]; then
                    print_success "Confirmed DB21J with 4GB RAM"
                    return 0
                else
                    print_warning "Detected Jetson Nano but only ${MEM_GB}GB RAM (DB21M?)"
                    print_warning "Continuing with reduced performance settings..."
                fi
            fi
        else
            print_error "Not running on Jetson hardware!"
            print_error "This deployment script is optimized for DB21J Duckiebot"
            exit 1
        fi
    else
        print_error "Cannot detect hardware model"
        exit 1
    fi
}

# Check and setup CUDA
setup_cuda() {
    print_info "Checking CUDA availability..."
    
    if command -v nvcc &> /dev/null; then
        CUDA_VERSION=$(nvcc --version | grep "release" | awk '{print $6}' | cut -d',' -f1)
        print_success "CUDA detected: $CUDA_VERSION"
        
        # Test CUDA with Python
        python3 -c "
import torch
if torch.cuda.is_available():
    print('✅ PyTorch CUDA available')
    print(f'🔥 GPU: {torch.cuda.get_device_name(0)}')
    print(f'💾 GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB')
else:
    print('⚠️ PyTorch CUDA not available')
" 2>/dev/null || print_warning "PyTorch not available or CUDA not working"
        
    else
        print_warning "CUDA not found - using CPU only mode"
    fi
}

# Apply Jetson performance optimizations
optimize_jetson_performance() {
    print_info "Applying Jetson Nano performance optimizations..."
    
    # Check if we have sudo access
    if sudo -n true 2>/dev/null; then
        # Set to maximum performance mode (0)
        if sudo nvpmodel -m 0 2>/dev/null; then
            print_success "Set Jetson to maximum performance mode"
        else
            print_warning "Could not set performance mode (may need manual setup)"
        fi
        
        # Enable maximum clocks
        if sudo jetson_clocks 2>/dev/null; then
            print_success "Enabled maximum clocks"
        else
            print_warning "Could not enable maximum clocks"
        fi
        
        # Check thermal throttling
        if [ -f /sys/devices/virtual/thermal/thermal_zone0/temp ]; then
            TEMP=$(cat /sys/devices/virtual/thermal/thermal_zone0/temp)
            TEMP_C=$((TEMP / 1000))
            if [ $TEMP_C -lt 70 ]; then
                print_success "Temperature OK: ${TEMP_C}°C"
            else
                print_warning "High temperature: ${TEMP_C}°C - consider cooling"
            fi
        fi
        
    else
        print_warning "No sudo access - some optimizations skipped"
        print_info "To enable full optimizations, run:"
        print_info "  sudo nvpmodel -m 0"
        print_info "  sudo jetson_clocks"
    fi
}

# Check ROS environment
check_ros_environment() {
    print_info "Checking ROS environment..."
    
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS not sourced! Please source your ROS environment:"
        print_error "  source /opt/ros/noetic/setup.bash"
        print_error "  source ~/catkin_ws/devel/setup.bash"
        exit 1
    else
        print_success "ROS environment: $ROS_DISTRO"
    fi
    
    if [ -z "$VEHICLE_NAME" ]; then
        print_warning "VEHICLE_NAME not set - using default"
        export VEHICLE_NAME="duckiebot"
    else
        print_success "Vehicle name: $VEHICLE_NAME"
    fi
}

# Build the dynamic obstacle avoidance package
build_package() {
    print_info "Building dynamic obstacle avoidance package..."
    
    cd ~/catkin_ws
    if catkin_make --pkg dynamic_obstacle_avoidance; then
        print_success "Package built successfully"
        source devel/setup.bash
    else
        print_error "Failed to build package"
        exit 1
    fi
}

# Test individual components
test_components() {
    print_info "Testing system components..."
    
    # Test camera
    print_info "Testing camera connection..."
    timeout 5 rostopic echo /camera_node/image/compressed/header -n 1 &>/dev/null && \
        print_success "Camera working" || print_warning "Camera not detected"
    
    # Test lane detection
    print_info "Testing lane filter..."
    timeout 5 rostopic echo /lane_filter_node/lane_pose -n 1 &>/dev/null && \
        print_success "Lane filter working" || print_warning "Lane filter not active"
    
    # Test existing object detection
    print_info "Testing object detection..."
    timeout 5 rostopic echo /object_detection_node/detections -n 1 &>/dev/null && \
        print_success "Object detection working" || print_warning "Object detection not active"
}

# Launch dynamic obstacle avoidance
launch_system() {
    local LAUNCH_TYPE=$1
    
    print_info "Launching dynamic obstacle avoidance system..."
    
    case $LAUNCH_TYPE in
        "jetson")
            print_info "Using Jetson-optimized configuration"
            roslaunch duckietown_demos lane_following_jetson_optimized.launch veh:=$VEHICLE_NAME
            ;;
        "standard")
            print_info "Using standard configuration"
            roslaunch duckietown_demos lane_following_with_dynamic_avoidance.launch veh:=$VEHICLE_NAME
            ;;
        "test")
            print_info "Testing individual components"
            roslaunch dynamic_obstacle_avoidance dynamic_obstacle_avoidance.launch veh:=$VEHICLE_NAME
            ;;
        *)
            print_error "Unknown launch type: $LAUNCH_TYPE"
            print_info "Available options: jetson, standard, test"
            exit 1
            ;;
    esac
}

# Print system information
print_system_info() {
    print_info "=== DB21J System Information ==="
    
    # Hardware info
    if [ -f /proc/device-tree/model ]; then
        echo "Hardware: $(cat /proc/device-tree/model)"
    fi
    
    # Memory info
    echo "Memory: $(free -h | grep Mem | awk '{print $2}')"
    
    # Disk space
    echo "Disk Space: $(df -h / | tail -1 | awk '{print $4}')"
    
    # CUDA info
    if command -v nvcc &> /dev/null; then
        echo "CUDA: $(nvcc --version | grep release | awk '{print $6}' | cut -d',' -f1)"
    else
        echo "CUDA: Not available"
    fi
    
    # ROS info
    echo "ROS Distro: ${ROS_DISTRO:-Not set}"
    echo "Vehicle: ${VEHICLE_NAME:-Not set}"
    
    # Temperature
    if [ -f /sys/devices/virtual/thermal/thermal_zone0/temp ]; then
        TEMP=$(cat /sys/devices/virtual/thermal/thermal_zone0/temp)
        TEMP_C=$((TEMP / 1000))
        echo "Temperature: ${TEMP_C}°C"
    fi
    
    print_info "=================================="
}

# Main execution
main() {
    echo "🤖 DB21J Dynamic Obstacle Avoidance Deployment Script"
    echo "======================================================"
    
    # Parse command line arguments
    COMMAND=${1:-"setup"}
    
    case $COMMAND in
        "setup")
            check_jetson_hardware
            setup_cuda
            optimize_jetson_performance
            check_ros_environment
            build_package
            print_success "Setup complete! Ready to launch."
            print_info "Usage examples:"
            print_info "  $0 launch jetson    # Launch with Jetson optimizations"
            print_info "  $0 launch standard  # Launch with standard settings"
            print_info "  $0 test            # Test components"
            print_info "  $0 info            # Show system information"
            ;;
        "launch")
            LAUNCH_TYPE=${2:-"jetson"}
            check_ros_environment
            launch_system $LAUNCH_TYPE
            ;;
        "test")
            check_ros_environment
            test_components
            ;;
        "info")
            print_system_info
            ;;
        "help"|"-h"|"--help")
            echo "Usage: $0 [command] [options]"
            echo ""
            echo "Commands:"
            echo "  setup          - Initial setup and optimization"
            echo "  launch [type]  - Launch system (jetson|standard|test)"
            echo "  test          - Test system components"
            echo "  info          - Show system information"
            echo "  help          - Show this help"
            echo ""
            echo "Examples:"
            echo "  $0 setup                    # Initial setup"
            echo "  $0 launch jetson           # Launch with Jetson optimizations"
            echo "  $0 launch standard         # Launch with standard settings"
            echo "  $0 test                    # Test components"
            ;;
        *)
            print_error "Unknown command: $COMMAND"
            print_info "Use '$0 help' for usage information"
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@" 