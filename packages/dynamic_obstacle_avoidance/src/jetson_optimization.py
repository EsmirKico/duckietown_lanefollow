#!/usr/bin/env python3
"""
Jetson Nano optimizations for DB21J Duckiebot
Provides hardware-specific performance tuning for dynamic obstacle avoidance
"""

import cv2
import numpy as np
import os
import logging

class JetsonOptimizer:
    """Hardware-specific optimizations for NVIDIA Jetson Nano on DB21J"""
    
    def __init__(self):
        self.is_jetson = self._detect_jetson()
        self.setup_logging()
        
        if self.is_jetson:
            self.optimize_jetson_performance()
            self.setup_gpu_memory()
            
    def _detect_jetson(self):
        """Detect if running on Jetson Nano"""
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read()
                return 'jetson' in model.lower()
        except:
            return False
            
    def setup_logging(self):
        """Setup optimized logging for Jetson"""
        if self.is_jetson:
            # Reduce logging overhead on limited hardware
            logging.basicConfig(level=logging.WARNING)
            
    def optimize_jetson_performance(self):
        """Apply Jetson Nano specific performance optimizations"""
        if not self.is_jetson:
            return
            
        try:
            # Set Jetson to max performance mode
            os.system('sudo nvpmodel -m 0')  # Max performance mode
            os.system('sudo jetson_clocks')  # Max clocks
            
            # Optimize OpenCV for ARM
            cv2.setUseOptimized(True)
            cv2.setNumThreads(4)  # Utilize all CPU cores
            
            print("✅ Jetson Nano optimizations applied")
            
        except Exception as e:
            print(f"⚠️ Could not apply all Jetson optimizations: {e}")
            
    def setup_gpu_memory(self):
        """Configure GPU memory for optimal performance"""
        if not self.is_jetson:
            return
            
        try:
            # Set GPU memory fraction for PyTorch if available
            import torch
            if torch.cuda.is_available():
                torch.cuda.set_per_process_memory_fraction(0.7)  # Reserve 70% for PyTorch
                print(f"🔥 CUDA available with {torch.cuda.get_device_name(0)}")
            else:
                print("⚠️ CUDA not available, using CPU only")
        except ImportError:
            print("⚠️ PyTorch not installed, skipping GPU optimization")
            
    def optimize_image_processing(self, enable_gpu=True):
        """Optimize OpenCV for Jetson Nano's GPU"""
        if not self.is_jetson:
            return {}
            
        optimization_params = {
            'use_gpu': enable_gpu and cv2.cuda.getCudaEnabledDeviceCount() > 0,
            'cpu_threads': 4,
            'memory_optimization': True
        }
        
        if optimization_params['use_gpu']:
            print("🚀 GPU-accelerated image processing enabled")
        else:
            print("🔄 Using optimized CPU image processing")
            
        return optimization_params
    
    def get_camera_optimization(self):
        """Get optimized camera parameters for DB21J"""
        return {
            'fps': 15,  # Balanced FPS for Jetson Nano
            'resolution': (640, 480),  # Optimal for processing speed
            'buffer_size': 1,  # Minimal buffering for real-time processing
            'fourcc': cv2.VideoWriter_fourcc(*'MJPG')  # Hardware-accelerated codec
        }
        
    def optimize_blob_detection(self):
        """Get optimized blob detection parameters for Jetson"""
        params = cv2.SimpleBlobDetector_Params()
        
        # Reduced complexity for real-time performance
        params.minThreshold = 50
        params.maxThreshold = 200
        params.filterByColor = True
        params.blobColor = 255
        params.filterByArea = True
        params.minArea = 30  # Slightly smaller for better detection
        params.maxArea = 5000
        params.filterByCircularity = True
        params.minCircularity = 0.6  # Relaxed for performance
        params.filterByConvexity = False  # Disable for speed
        params.filterByInertia = True
        params.minInertiaRatio = 0.4  # Relaxed for performance
        
        return params
        
    def memory_cleanup(self):
        """Perform memory cleanup optimized for limited RAM"""
        if self.is_jetson:
            import gc
            gc.collect()
            
            # Clear OpenCV cache
            try:
                cv2.destroyAllWindows()
            except:
                pass


# Global optimizer instance
jetson_optimizer = JetsonOptimizer()


def apply_jetson_optimizations():
    """Apply all Jetson optimizations at startup"""
    return jetson_optimizer.optimize_image_processing()


def get_optimized_camera_params():
    """Get camera parameters optimized for DB21J"""
    return jetson_optimizer.get_camera_optimization()


def get_optimized_blob_params():
    """Get blob detection parameters optimized for Jetson Nano"""
    return jetson_optimizer.optimize_blob_detection()


def cleanup_memory():
    """Cleanup memory for optimal performance"""
    jetson_optimizer.memory_cleanup() 