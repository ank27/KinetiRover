#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from pcl_ros import transforms

class PointCloudProcessor:
    def __init__(self):
        rospy.init_node('point_cloud_processor', anonymous=True)
        
        # Get parameters
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.min_height = rospy.get_param('~min_height', 0.01)
        self.max_height = rospy.get_param('~max_height', 0.5)
        self.voxel_size = rospy.get_param('~voxel_size', 0.01)
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        
        # Publishers and subscribers
        self.cloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.cloud_callback)
        self.processed_pub = rospy.Publisher('/processed_points', PointCloud2, queue_size=1)
        
    def filter_points(self, points):
        """Filter points based on height and remove outliers"""
        mask = np.logical_and(points[:, 2] >= self.min_height,
                            points[:, 2] <= self.max_height)
        return points[mask]
    
    def voxel_grid_filter(self, points):
        """Downsample point cloud using voxel grid filter"""
        voxel_coords = np.floor(points / self.voxel_size)
        _, unique_indices = np.unique(voxel_coords, axis=0, return_index=True)
        return points[unique_indices]
    
    def cloud_callback(self, msg):
        try:
            # Transform point cloud to base frame
            cloud_transformed = transforms.transformPointCloud(self.base_frame, msg, self.tf_buffer)
            
            # Convert to numpy array
            points = np.array(list(pc2.read_points(cloud_transformed, 
                                                 field_names=("x", "y", "z"))))
            
            # Filter and downsample points
            filtered_points = self.filter_points(points)
            downsampled_points = self.voxel_grid_filter(filtered_points)
            
            # Convert back to PointCloud2 and publish
            header = msg.header
            header.frame_id = self.base_frame
            processed_msg = pc2.create_cloud_xyz32(header, downsampled_points)
            self.processed_pub.publish(processed_msg)
            
        except (tf2_geometry_msgs.tf2_geometry_msgs.TransformException, 
                rospy.ROSException) as e:
            rospy.logwarn(f"Failed to process point cloud: {e}")

if __name__ == '__main__':
    try:
        processor = PointCloudProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass