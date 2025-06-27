#!/usr/bin/env python3
"""
CUDA utilities for Jetson Nano optimization in Safe Navigation project.
"""

import os
import torch
import logging

logger = logging.getLogger(__name__)


def setup_jetson_cuda():
    """Setup CUDA optimizations for Jetson Nano."""
    # Set CUDA memory management for Jetson Nano 4GB
    os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128'
    os.environ['CUDA_LAUNCH_BLOCKING'] = '1'
    
    # Optimize CPU threads for Jetson Nano (4 cores)
    torch.set_num_threads(4)
    os.environ['OMP_NUM_THREADS'] = '4'
    
    if torch.cuda.is_available():
        # Set memory fraction for Jetson Nano
        torch.cuda.set_per_process_memory_fraction(0.8)
        torch.cuda.empty_cache()
        logger.info(f"CUDA setup complete for {torch.cuda.get_device_name(0)}")
    else:
        logger.warning("CUDA not available")


def get_optimal_device():
    """Get the optimal device for computation."""
    return torch.device('cuda' if torch.cuda.is_available() else 'cpu')


def optimize_model_for_jetson(model):
    """Optimize PyTorch model for Jetson Nano inference."""
    model.eval()
    
    if torch.cuda.is_available():
        model = model.cuda()
        # Enable cuDNN optimizations
        torch.backends.cudnn.benchmark = True
        torch.backends.cudnn.enabled = True
    
    return model


def verify_cuda_setup():
    """Verify CUDA setup."""
    print("=== CUDA Setup Verification ===")
    print(f"PyTorch Version: {torch.__version__}")
    print(f"CUDA Available: {torch.cuda.is_available()}")
    
    if torch.cuda.is_available():
        print(f"CUDA Version: {torch.version.cuda}")
        print(f"Device: {torch.cuda.get_device_name(0)}")
        print(f"Total Memory: {torch.cuda.get_device_properties(0).total_memory // 1024 // 1024} MB")
        return True
    else:
        print("CUDA not available")
        return False


if __name__ == "__main__":
    verify_cuda_setup() 