#!/usr/bin/env python3
import cv2
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Float64MultiArray
try:
    from .jetson_optimization import apply_jetson_optimizations, get_optimized_blob_params, cleanup_memory
except ImportError:
    # Fallback import for when running as script
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from jetson_optimization import apply_jetson_optimizations, get_optimized_blob_params, cleanup_memory


class LEDDetectionNode(DTROS):

    def __init__(self, node_name):
        super(LEDDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        self.bridge = CvBridge()
        self.active = True
        self.publish_freq = rospy.get_param("~publish_freq", 2.0)
        self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.last_stamp = rospy.Time.now()
        
        # Apply Jetson Nano optimizations
        self.optimization_params = apply_jetson_optimizations()
        
        # Simplified camera parameters - will be updated with actual values
        self.fx = 320.0  # fallback focal length
        self.fy = 320.0  # fallback focal length
        self.cx = 320.0  # fallback center x
        self.cy = 240.0  # fallback center y

        # Blue threshold for distinguishing red vs white LEDs
        self.blue_threshold = 235

        # Subscribers
        self.sub_image = rospy.Subscriber("~image", CompressedImage,
                                          self.processImage, buff_size=921600,
                                          queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped,
                                           self.cbSwitch, queue_size=1)

        # Publishers
        self.pub_circlepattern_image = rospy.Publisher("~circlepattern_image",
                                                       Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                                Float32, queue_size=1)

        self.pub_detected_duckiebot_head = rospy.Publisher("~detected_duckiebot_head",
                                                Float64MultiArray, queue_size=1)
        self.pub_detected_duckiebot_tail = rospy.Publisher("~detected_duckiebot_tail",
                                                Float64MultiArray, queue_size=1)

        self.pub_detected_duckiebot_head_state = rospy.Publisher("~detected_duckiebot_head_state",
                                             BoolStamped, queue_size=1)

        self.pub_detected_duckiebot_tail_state = rospy.Publisher("~detected_duckiebot_tail_state",
                                             BoolStamped, queue_size=1)

        self.log("LED Detection Node initialized!")

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def features_deepcopy(self, keypoints):
        """Deep copy keypoints because deepcopy doesn't work with cv2.KeyPoint"""
        return [cv2.KeyPoint(x=k.pt[0], y=k.pt[1],
                size=k.size, angle=k.angle,
                response=k.response, octave=k.octave,
                class_id=k.class_id) for k in keypoints]

    def processImage(self, image_msg):
        if not self.active:
            return

        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now

        try:
            cv_image_color = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            self.logerr(f"Could not decode image: {e}")
            return

        start = rospy.Time.now()

        # Crop top and bottom image as the LEDs can not be here
        height = cv_image_color.shape[0]
        crop_top = int(height * 0.3)
        crop_bottom = int(height * 0.75)
        cv_image_color = cv_image_color[crop_top:crop_bottom]

        # Convert to grayscale
        cv_image_gray = cv2.cvtColor(cv_image_color, cv2.COLOR_BGR2GRAY)

        # Binary image based on high threshold to get bright parts (bright LEDs etc.)
        _, cv_image_binary = cv2.threshold(cv_image_gray, 220, 255, cv2.THRESH_BINARY)

        # Set up the blob detector with Jetson optimizations
        try:
            params = get_optimized_blob_params()
            # Override some parameters for LED-specific detection
            params.filterByCircularity = True
            params.minCircularity = 0.7  # LEDs are close to circular
            params.filterByArea = True
            params.minArea = 50  # Smaller minimum for distant LEDs
        except:
            # Fallback parameters if optimization fails
            params = cv2.SimpleBlobDetector_Params()
            params.minThreshold = 10
            params.maxThreshold = 200
            params.filterByColor = True
            params.blobColor = 255
            params.filterByCircularity = True
            params.minCircularity = 0.7
        
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs
        keypoints = detector.detect(cv_image_binary)

        # Initialize detection states
        white_found = False
        red_found = False
        white_pose = [0.0, 0.0]
        red_pose = [0.0, 0.0]

        # Try to find the two keypoints that are the 2 LEDs
        for i, key1 in enumerate(keypoints):
            for j, key2 in enumerate(keypoints):
                if i != j:
                    # Check if roughly same size
                    if abs((key1.size - key2.size) / key1.size) < 0.4:
                        # Check if roughly same y coordinate
                        if abs(key1.pt[1] - key2.pt[1]) < key1.size:
                            # Get color of the keypoint centers
                            try:
                                pixel1 = cv_image_color[int(key1.pt[1]), int(key1.pt[0])]
                                pixel2 = cv_image_color[int(key2.pt[1]), int(key2.pt[0])]
                                blue1 = pixel1[0]
                                blue2 = pixel2[0]

                                # Check if the blue value matches red back or white front
                                if blue1 < self.blue_threshold and blue2 < self.blue_threshold:  # Red LEDs
                                    red_found = True
                                    # Calculate center position
                                    center_x = (key1.pt[0] + key2.pt[0]) / 2
                                    center_y = (key1.pt[1] + key2.pt[1]) / 2
                                    # Simple distance estimation (simplified)
                                    distance = max(0.1, 100.0 / max(key1.size, 1.0))  # Very rough estimate
                                    red_pose = [distance, 0.0]  # x, y in robot frame

                                elif blue1 >= self.blue_threshold and blue2 >= self.blue_threshold:  # White LEDs
                                    white_found = True
                                    # Calculate center position
                                    center_x = (key1.pt[0] + key2.pt[0]) / 2
                                    center_y = (key1.pt[1] + key2.pt[1]) / 2
                                    # Simple distance estimation (simplified)
                                    distance = max(0.1, 100.0 / max(key1.size, 1.0))  # Very rough estimate
                                    white_pose = [distance, 0.0]  # x, y in robot frame

                            except (IndexError, ValueError):
                                continue

        # Publish detection states
        head_state_msg = BoolStamped()
        head_state_msg.header.stamp = rospy.Time.now()
        head_state_msg.data = white_found
        self.pub_detected_duckiebot_head_state.publish(head_state_msg)

        tail_state_msg = BoolStamped()
        tail_state_msg.header.stamp = rospy.Time.now()
        tail_state_msg.data = red_found
        self.pub_detected_duckiebot_tail_state.publish(tail_state_msg)

        # Publish pose data
        if white_found:
            head_pose_msg = Float64MultiArray()
            head_pose_msg.data = white_pose
            self.pub_detected_duckiebot_head.publish(head_pose_msg)

        if red_found:
            tail_pose_msg = Float64MultiArray()
            tail_pose_msg.data = red_pose
            self.pub_detected_duckiebot_tail.publish(tail_pose_msg)

        # Publish debug image with keypoints drawn
        if len(keypoints) > 0:
            debug_image = cv2.drawKeypoints(cv_image_color, keypoints, cv_image_color, 
                                          color=(0, 0, 255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                self.pub_circlepattern_image.publish(debug_msg)
            except CvBridgeError as e:
                self.logerr(f"Could not publish debug image: {e}")

        # Publish processing time
        elapsed_time = (rospy.Time.now() - start).to_sec()
        time_msg = Float32()
        time_msg.data = elapsed_time
        self.pub_time_elapsed.publish(time_msg)
        
        # Periodic memory cleanup for Jetson Nano
        if hasattr(self, 'frame_count'):
            self.frame_count += 1
        else:
            self.frame_count = 1
            
        if self.frame_count % 100 == 0:  # Cleanup every 100 frames
            cleanup_memory()


if __name__ == '__main__':
    node = LEDDetectionNode(node_name='led_detection_node')
    rospy.spin() 