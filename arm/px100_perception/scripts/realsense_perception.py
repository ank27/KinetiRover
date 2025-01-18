#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import message_filters


class RealsensePerception:
    def __init__(self):
        rospy.init_node('realsense_perception_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Get parameters
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # Initialize TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize subscribers with time synchronization
        self.color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self.pc_sub = message_filters.Subscriber('/camera/depth/color/points', PointCloud2)
        
        # Time synchronizer
        ts = message_filters.TimeSynchronizer([self.color_sub, self.depth_sub, self.pc_sub], 10)
        ts.registerCallback(self.perception_callback)
        
        # Publishers
        self.processed_img_pub = rospy.Publisher('~processed_image', Image, queue_size=1)
        self.object_pose_pub = rospy.Publisher('~object_pose', PoseStamped, queue_size=1)
        
        rospy.loginfo("RealSense perception node initialized")

    def perception_callback(self, color_msg, depth_msg, pc_msg):
        try:
            # Convert ROS images to OpenCV format
            color_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            
            # Basic image processing (example)
            processed_img = self.process_image(color_img)
            
            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_img, "bgr8")
            self.processed_img_pub.publish(processed_msg)
            
        except Exception as e:
            rospy.logerr(f"Error in perception callback: {str(e)}")

    def process_image(self, img):
        """
        Basic image processing pipeline
        Add your custom processing steps here
        """
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply adaptive thresholding
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
        )
        
        # Find contours
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Draw contours on original image
        result = img.copy()
        cv2.drawContours(result, contours, -1, (0, 255, 0), 2)
        
        return result

    def transform_pose(self, pose_stamped):
        """
        Transform pose from camera frame to base frame
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                pose_stamped.header.frame_id,
                pose_stamped.header.stamp,
                rospy.Duration(1.0)
            )
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF Error: {str(e)}")
            return None

if __name__ == '__main__':
    try:
        node = RealsensePerception()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
