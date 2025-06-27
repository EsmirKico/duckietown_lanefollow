#!/bin/bash

# Safe Navigation CUDA Deployment Script for Jetson Nano
# Based on Duckietown Shell (DTS) best practices

set -e

# Configuration
REPO_NAME="safe-navigation"
IMAGE_NAME="duckietown/${REPO_NAME}"
ARCH="arm64v8"
TAG="my-${ARCH}"
VEHICLE_NAME=${VEHICLE_NAME:-"ducky"}

echo "🚀 Safe Navigation CUDA Deployment for Jetson Nano"
echo "=================================================="
echo "Vehicle: ${VEHICLE_NAME}"
echo "Architecture: ${ARCH}"
echo "Image: ${IMAGE_NAME}:${TAG}"
echo ""

# Check if running on Jetson
if [ -f /proc/device-tree/model ]; then
    MODEL=$(cat /proc/device-tree/model 2>/dev/null || echo "unknown")
    if [[ $MODEL == *"jetson"* ]]; then
        echo "✓ Detected Jetson device: $MODEL"
    else
        echo "⚠️  Not running on Jetson device"
    fi
fi

# Check CUDA availability
echo "🔍 Checking CUDA setup..."
if command -v nvidia-smi &> /dev/null; then
    echo "✓ nvidia-smi available"
    nvidia-smi --query-gpu=name,memory.total,memory.free --format=csv,noheader,nounits
else
    echo "❌ nvidia-smi not found - CUDA may not be available"
fi

# Check Docker runtime
if docker info | grep -q nvidia; then
    echo "✓ NVIDIA Docker runtime available"
else
    echo "⚠️  NVIDIA Docker runtime not detected"
fi

echo ""
echo "🔨 Building Docker image with CUDA support..."

# Build the image
dts devel build -f --arch ${ARCH} -H ${VEHICLE_NAME}.local

if [ $? -eq 0 ]; then
    echo "✅ Build completed successfully"
else
    echo "❌ Build failed"
    exit 1
fi

echo ""
echo "🧪 Testing CUDA in container..."

# Test CUDA functionality
docker run --rm --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e VEHICLE_NAME=${VEHICLE_NAME} \
    ${IMAGE_NAME}:${TAG} \
    python3 -c "
import torch
print('PyTorch version:', torch.__version__)
print('CUDA available:', torch.cuda.is_available())
if torch.cuda.is_available():
    print('CUDA version:', torch.version.cuda)
    print('Device count:', torch.cuda.device_count())
    print('Device name:', torch.cuda.get_device_name(0))
    print('Memory total:', torch.cuda.get_device_properties(0).total_memory // 1024**2, 'MB')
    # Test basic operation
    x = torch.randn(100, 100).cuda()
    y = torch.randn(100, 100).cuda()
    z = torch.mm(x, y)
    print('✓ Basic CUDA operations working')
else:
    print('❌ CUDA not available in container')
"

echo ""
echo "🚀 Running Safe Navigation with CUDA support..."

# Run the container with full CUDA support
dts devel run -H ${VEHICLE_NAME}.local --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video \
    -e VEHICLE_NAME=${VEHICLE_NAME} \
    -e PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128 \
    -e JETSON_PERFORMANCE_MODE=1 \
    -e JETSON_FP16=true \
    --shm-size=2g \
    --memory=3.5g

echo ""
echo "✅ Deployment complete!"
echo ""
echo "📋 Next steps:"
echo "1. Monitor GPU usage: nvidia-smi"
echo "2. Check container logs: docker logs <container_id>"
echo "3. Test object detection performance"
echo "4. Monitor memory usage and temperature" 