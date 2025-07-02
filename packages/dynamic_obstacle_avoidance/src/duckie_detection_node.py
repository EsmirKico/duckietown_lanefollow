#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped, Pixel, LanePose
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Float64MultiArray
from dynamic_obstacle_avoidance.msg import dynamic_obstacle
try:
    from .jetson_optimization import apply_jetson_optimizations, get_optimized_blob_params, cleanup_memory
except ImportError:
    # Fallback import for when running as script
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from jetson_optimization import apply_jetson_optimizations, get_optimized_blob_params, cleanup_memory


class DuckieDetectionNode(DTROS):

    def __init__(self, node_name):
        super(DuckieDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        self.bridge = CvBridge()
        self.active = True
        self.publish_freq = rospy.get_param("~publish_freq", 2.0)
        self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.last_stamp = rospy.Time.now()
        
        # Apply Jetson Nano optimizations
        self.optimization_params = apply_jetson_optimizations()

        self.publish_debugimg = True  # boolean to publish debug image (boundingbox and mask)
        self.lane_width = 0.1145  # half of lane
        self.crop_factor = 0.3  # percentage of image cropped from the top
        self.d = 0.0
        self.phi = 0.0
        self.duckie_pos_arr_prev = []

        # Simplified camera parameters - will be updated with actual values if available
        self.resolution = np.array([640, 480])  # fallback resolution

        # HSV range for yellow duckies
        self.yellow_low = np.array([25, 180, 180])
        self.yellow_high = np.array([35, 255, 255])

        # Simplified homography matrix (identity as fallback)
        self.H = np.array([[1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0],
                          [0.0, 0.0, 1.0]])

        # Subscribers
        self.sub_image = rospy.Subscriber("~image", CompressedImage, self.processImage,
                                          buff_size=921600, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped,
                                          self.cbSwitch, queue_size=1)
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.cbLanePose, queue_size=1)

        # Publishers
        self.pub_boundingbox_image = rospy.Publisher("~duckiedetected_image",
                                                    Image, queue_size=1)
        self.pub_mask_image = rospy.Publisher("~duckiedetected_mask",
                                             Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                               Float32, queue_size=1)
        self.pub_duckie = rospy.Publisher("~detected_duckie",
                                         dynamic_obstacle, queue_size=1)

        self.log("Duckie Detection Node initialized!")

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def cbLanePose(self, lane_pose_msg):
        """Callback to lane pose"""
        self.d = lane_pose_msg.d
        self.phi = lane_pose_msg.phi

    def pixel2ground(self, pixel):
        """Convert pixel coordinates to ground coordinates using simplified transformation"""
        uv_raw = np.array([pixel.u, pixel.v, 1.0])
        # Simple transformation - in real implementation this would use proper homography
        # For now, just convert pixel to approximate ground coordinates
        ground_point = np.dot(self.H, uv_raw)
        
        point = Point()
        if ground_point[2] != 0:
            point.x = ground_point[0] / ground_point[2] * 0.001  # convert to meters (very rough)
            point.y = ground_point[1] / ground_point[2] * 0.001
        else:
            point.x = 0.0
            point.y = 0.0
        point.z = 0.0
        
        return point

    def processImage(self, image_msg):
        if not self.active:
            return

        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            self.logerr(f"Could not decode image: {e}")
            return

        start = rospy.Time.now()

        # Update resolution from actual image
        self.resolution = np.array([cv_image.shape[1], cv_image.shape[0]])

        # Crop upper percentage and lower 1/4 of image
        crop_top = int(cv_image.shape[0] * self.crop_factor)
        crop_bottom = int(cv_image.shape[0] * 3/4)
        cv_image_crop = cv_image[crop_top:crop_bottom, :]
        
        hsv_img = cv2.cvtColor(cv_image_crop, cv2.COLOR_BGR2HSV)

        # Create a yellow mask
        mask = cv2.inRange(hsv_img, self.yellow_low, self.yellow_high)

        # Initialize arrays
        duckiefound = False
        duckie_loc_pix = Pixel()
        duckie_msg = dynamic_obstacle()
        duckie_pos_arr = []
        duckie_pos_arr_new = []
        duckie_state_arr = []

        # Initialize parameters for blob detector with Jetson optimizations
        # minInertiaRatio is especially important, it filters out the elongated lane segments
        try:
            params = get_optimized_blob_params()
            # Override for duckie-specific detection
            params.filterByInertia = True
            params.minInertiaRatio = 0.5  # Filter out elongated lane segments
            params.filterByCircularity = False  # Duckies aren't perfectly circular
            params.minArea = 30  # Smaller minimum for distant duckies
        except:
            # Fallback parameters if optimization fails
            params = cv2.SimpleBlobDetector_Params()
            params.filterByColor = True
            params.blobColor = 255
            params.filterByArea = True
            params.minArea = 40
            params.filterByInertia = True
            params.minInertiaRatio = 0.5
            params.filterByConvexity = False
            params.filterByCircularity = False
        
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs and draw them in image and in mask
        keypoints = detector.detect(mask)
        cv_image_crop = cv2.drawKeypoints(cv_image_crop, keypoints, cv_image_crop, 
                                         color=(0, 0, 255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        mask = cv2.drawKeypoints(mask, keypoints, mask, 
                                color=(255, 255, 255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Loop to check location of duckies to see if they are in the lane
        if keypoints:
            duckiefound = True
            for key in keypoints:
                duckie_loc_pix.u = key.pt[0]
                duckie_loc_pix.v = key.pt[1] + key.size/2 + float(self.resolution[1]) * self.crop_factor

                # Project duckie location to ground
                duckie_loc_world = self.pixel2ground(duckie_loc_pix)
                duckie_pos_arr_new.append([duckie_loc_world.x, duckie_loc_world.y])

                # Calculate distance of duckie to the middle line
                duckie_side = np.cos(self.phi) * (duckie_loc_world.y + self.d) + np.sin(self.phi) * duckie_loc_world.x

                # Tracking for outlier prevention: add duckie only if it has been detected already previously close by
                prev_detected = []
                for a in range(len(self.duckie_pos_arr_prev)):
                    dist = np.sqrt((duckie_loc_world.x - self.duckie_pos_arr_prev[a][0])**2 + 
                                  (duckie_loc_world.y - self.duckie_pos_arr_prev[a][1])**2)
                    prev_detected.append(dist < 0.1)

                # Check if duckie is on the left or right lane
                if any(prev_detected) or len(self.duckie_pos_arr_prev) == 0:  # Allow first detection
                    if abs(duckie_side) < self.lane_width:  # right lane
                        duckie_state_arr.append(1)
                        duckie_pos_arr.extend([duckie_loc_world.x, duckie_loc_world.y])
                    elif duckie_side > self.lane_width and duckie_side < self.lane_width * 3:  # left lane
                        duckie_state_arr.append(2)
                        duckie_pos_arr.extend([duckie_loc_world.x, duckie_loc_world.y])

        self.duckie_pos_arr_prev = duckie_pos_arr_new

        if not duckie_state_arr:
            duckie_state_arr.append(0)  # write zero if no duckie detected

        # Fill information into the duckie_msg and publish it
        duckie_msg.pos = duckie_pos_arr
        duckie_msg.state = duckie_state_arr
        duckie_msg.header.stamp = rospy.Time.now()
        self.pub_duckie.publish(duckie_msg)

        # If boolean is on True: publish debug images (mask and hsv image)
        if self.publish_debugimg:
            try:
                image_msg_out = self.bridge.cv2_to_imgmsg(cv_image_crop, "bgr8")
                image_mask_msg_out = self.bridge.cv2_to_imgmsg(mask, "mono8")
                self.pub_boundingbox_image.publish(image_msg_out)
                self.pub_mask_image.publish(image_mask_msg_out)
            except CvBridgeError as e:
                self.logerr(f"Could not publish debug images: {e}")

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
    node = DuckieDetectionNode(node_name='duckie_detection_node')
    rospy.spin() 