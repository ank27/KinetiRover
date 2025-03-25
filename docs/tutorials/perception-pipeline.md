# Perception Pipeline for KinetiRover

*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*

## Introduction

The perception pipeline is a critical component of any robotic system that interacts with its environment. For the KinetiRover, the perception pipeline enables the robot to identify objects, determine their positions in 3D space, and ultimately perform pick-and-place operations.

In this tutorial, we'll cover:
1. Setting up the perception pipeline components
2. Point cloud processing techniques
3. Object detection with YOLO
4. Combining 2D detection with 3D point clouds
5. Integration with MoveIt for pick-and-place tasks

## Prerequisites

- KinetiRover base setup completed
- RealSense D435 camera configured
- ROS Noetic installed
- Basic understanding of Python programming

## 1. Perception Pipeline Architecture

Our perception pipeline consists of several components:

```
+-----------------+     +----------------+     +---------------+
| RealSense D435  | --> | Point Cloud    | --> | Segmentation  |
| Camera          |     | Preprocessing  |     |               |
+-----------------+     +----------------+     +---------------+
                                                      |
+-----------------+     +----------------+     +---------------+
| Pick & Place    | <-- | 3D Position    | <-- | Object        |
| with MoveIt     |     | Estimation     |     | Detection     |
+-----------------+     +----------------+     +---------------+
```

## 2. Point Cloud Processing

### 2.1 Installing Required Packages

```bash
sudo apt-get install ros-noetic-pcl-ros ros-noetic-pcl-conversions
pip install opencv-python numpy
```

### 2.2 Creating a Point Cloud Filtering Node

Create a file named `point_cloud_filter.py` in your package:

```python
#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from pcl_msgs.msg import PointIndices
import pcl
import pcl_ros

class PointCloudFilter:
    def __init__(self):
        rospy.init_node('point_cloud_filter')
        
        # Parameters
        self.voxel_size = rospy.get_param('~voxel_size', 0.01)  # 1cm voxel
        self.z_min = rospy.get_param('~z_min', 0.05)  # 5cm above table
        self.z_max = rospy.get_param('~z_max', 0.5)   # 50cm height limit
        
        # Subscribers and Publishers
        self.cloud_sub = rospy.Subscriber('/camera/depth/color/points', 
                                         PointCloud2, self.cloud_callback)
        self.cloud_pub = rospy.Publisher('/filtered_points', 
                                         PointCloud2, queue_size=1)
    
    def cloud_callback(self, cloud_msg):
        # Convert ROS message to PCL
        cloud = pcl_ros.point_cloud2.create_cloud_xyz32(cloud_msg.header, 
            [[p[0], p[1], p[2]] for p in pc2.read_points(cloud_msg)])
        
        # Apply voxel grid filter
        voxel_filter = cloud.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(self.voxel_size, self.voxel_size, self.voxel_size)
        cloud_filtered = voxel_filter.filter()
        
        # Apply passthrough filter for z height
        passthrough = cloud_filtered.make_passthrough_filter()
        passthrough.set_filter_field_name("z")
        passthrough.set_filter_limits(self.z_min, self.z_max)
        cloud_filtered = passthrough.filter()
        
        # Convert back to ROS message and publish
        filtered_msg = pcl_ros.point_cloud2.create_cloud_xyz32(
            cloud_msg.header, cloud_filtered.to_array())
        self.cloud_pub.publish(filtered_msg)

if __name__ == '__main__':
    try:
        filter_node = PointCloudFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

### 2.3 Segmentation Techniques

We'll use plane segmentation to identify the table surface and cluster extraction to identify objects:

```python
#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from pcl_msgs.msg import PointIndices
import pcl
import pcl_ros
from visualization_msgs.msg import Marker, MarkerArray

class PointCloudSegmenter:
    def __init__(self):
        rospy.init_node('point_cloud_segmenter')
        
        # Parameters
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.01)
        self.cluster_tolerance = rospy.get_param('~cluster_tolerance', 0.02)
        self.min_cluster_size = rospy.get_param('~min_cluster_size', 100)
        
        # Subscribers and Publishers
        self.cloud_sub = rospy.Subscriber('/filtered_points', 
                                          PointCloud2, self.segmentation_callback)
        self.clusters_pub = rospy.Publisher('/object_clusters', 
                                          PointCloud2, queue_size=1)
        self.markers_pub = rospy.Publisher('/object_markers', 
                                           MarkerArray, queue_size=1)
    
    def segmentation_callback(self, cloud_msg):
        # Convert ROS message to PCL
        cloud = pcl_ros.point_cloud2.create_cloud_xyz32(cloud_msg.header, 
            [[p[0], p[1], p[2]] for p in pc2.read_points(cloud_msg)])
        
        # Plane segmentation (to remove table)
        seg = cloud.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_normal_distance_weight(0.1)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(100)
        seg.set_distance_threshold(self.distance_threshold)
        indices, coefficients = seg.segment()
        
        # Extract non-plane points
        cloud_objects = cloud.extract(indices, negative=True)
        
        # Cluster extraction
        tree = cloud_objects.make_kdtree()
        ec = cloud_objects.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(self.cluster_tolerance)
        ec.set_MinClusterSize(self.min_cluster_size)
        ec.set_MaxClusterSize(25000)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()
        
        # Publish cluster markers
        marker_array = MarkerArray()
        for i, indices in enumerate(cluster_indices):
            # Extract cluster points
            points = np.zeros((len(indices), 3), dtype=np.float32)
            for j, idx in enumerate(indices):
                points[j] = cloud_objects[idx]
            
            # Compute centroid
            centroid = np.mean(points, axis=0)
            
            # Create marker
            marker = Marker()
            marker.header = cloud_msg.header
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = centroid[0]
            marker.pose.position.y = centroid[1]
            marker.pose.position.z = centroid[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime = rospy.Duration(1.0)
            marker_array.markers.append(marker)
        
        self.markers_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        segmenter = PointCloudSegmenter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

## 3. Object Detection with YOLO

### 3.1 Setting Up YOLO for ROS

First, install the required packages:

```bash
# Install Darknet ROS
cd ~/kinetibot_ws/src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git

# Build the workspace
cd ~/kinetibot_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 3.2 Creating a Custom YOLO Node

Create a Python node to process YOLO detections and match them with point cloud clusters:

```python
#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Point, PoseStamped
import tf2_ros
import tf2_geometry_msgs

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector')
        self.bridge = CvBridge()
        
        # Store the latest data
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_boxes = None
        self.camera_info = None
        
        # TF buffer for transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Subscribe to camera topics
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', 
                                       Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', 
                                         Image, self.depth_callback)
        self.info_sub = rospy.Subscriber('/camera/color/camera_info', 
                                        CameraInfo, self.info_callback)
        
        # Subscribe to YOLO detection results
        self.bbox_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', 
                                        BoundingBoxes, self.bbox_callback)
        
        # Publisher for object poses
        self.pose_pub = rospy.Publisher('/detected_objects/poses', 
                                       PoseStamped, queue_size=10)
                                       
        rospy.loginfo("Object detector initialized")
    
    def rgb_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        # Convert to meters
        self.latest_depth = self.latest_depth.astype(float) / 1000.0
    
    def info_callback(self, msg):
        self.camera_info = msg
    
    def bbox_callback(self, msg):
        if self.latest_rgb is None or self.latest_depth is None or self.camera_info is None:
            return
        
        # Process each detected object
        for box in msg.bounding_boxes:
            # Get center of bounding box
            center_x = (box.xmin + box.xmax) // 2
            center_y = (box.ymin + box.ymax) // 2
            
            # Get depth at center point
            depth = self.latest_depth[center_y, center_x]
            
            # Skip if depth is invalid
            if depth <= 0 or np.isnan(depth):
                continue
            
            # Convert pixel to 3D point
            fx = self.camera_info.K[0]
            fy = self.camera_info.K[4]
            cx = self.camera_info.K[2]
            cy = self.camera_info.K[5]
            
            # Calculate 3D point in camera frame
            x = (center_x - cx) * depth / fx
            y = (center_y - cy) * depth / fy
            z = depth
            
            # Create pose message
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            
            # Transform to base_link frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    msg.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
                
                # Publish the transformed pose
                self.pose_pub.publish(pose_transformed)
                
                rospy.loginfo(f"Detected {box.Class} at position: "
                             f"[{pose_transformed.pose.position.x:.3f}, "
                             f"{pose_transformed.pose.position.y:.3f}, "
                             f"{pose_transformed.pose.position.z:.3f}]")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF Error: {e}")

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

### 3.3 Configuring YOLO

Create a launch file for the object detection system:

```xml
<launch>
    <!-- Start RealSense camera -->
    <include file="$(find realsense2_camera)/launch/rs_camera.launch">
        <arg name="align_depth" value="true"/>
    </include>

    <!-- Start Darknet ROS with YOLO -->
    <include file="$(find darknet_ros)/launch/yolo_v4.launch">
        <arg name="image" value="/camera/color/image_raw"/>
    </include>
    
    <!-- Start point cloud filter -->
    <node name="point_cloud_filter" pkg="kinetirover_perception" type="point_cloud_filter.py" output="screen">
        <param name="voxel_size" value="0.01"/>
        <param name="z_min" value="0.05"/>
        <param name="z_max" value="0.5"/>
    </node>
    
    <!-- Start point cloud segmenter -->
    <node name="point_cloud_segmenter" pkg="kinetirover_perception" type="point_cloud_segmenter.py" output="screen">
        <param name="distance_threshold" value="0.01"/>
        <param name="cluster_tolerance" value="0.02"/>
        <param name="min_cluster_size" value="100"/>
    </node>
    
    <!-- Start object detector -->
    <node name="object_detector" pkg="kinetirover_perception" type="object_detector.py" output="screen"/>
</launch>
```

## 4. Integrating with MoveIt for Pick-and-Place

### 4.1 Creating a Pick-and-Place Node

Now, let's create a node that uses the detected object poses to perform pick-and-place operations with MoveIt:

```python
#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler
import numpy as np

class PickAndPlace:
    def __init__(self):
        rospy.init_node('pick_and_place')
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "px100_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.gripper_group = moveit_commander.MoveGroupCommander("px100_gripper")
        
        # Set parameters
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        
        # Add table to the scene
        self.add_table()
        
        # Subscribe to detected object poses
        self.object_pose_sub = rospy.Subscriber('/detected_objects/poses', 
                                               PoseStamped, self.object_callback)
        
        rospy.loginfo("Pick and place node initialized")
    
    def add_table(self):
        # Add table as a collision object
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0.2  # 20cm in front of the robot
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -0.025  # Half of the table height
        table_pose.pose.orientation.w = 1.0
        
        self.scene.add_box("table", table_pose, size=(0.5, 0.5, 0.05))
        rospy.sleep(1.0)  # Wait for the scene to update
    
    def open_gripper(self):
        gripper_joint_values = self.gripper_group.get_current_joint_values()
        gripper_joint_values[0] = 0.037  # Open position for left_finger
        self.gripper_group.go(gripper_joint_values, wait=True)
        self.gripper_group.stop()
    
    def close_gripper(self):
        gripper_joint_values = self.gripper_group.get_current_joint_values()
        gripper_joint_values[0] = 0.015  # Closed position for left_finger
        self.gripper_group.go(gripper_joint_values, wait=True)
        self.gripper_group.stop()
    
    def move_to_home(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0.0  # waist
        joint_goal[1] = 0.0  # shoulder
        joint_goal[2] = 0.0  # elbow
        joint_goal[3] = 0.0  # wrist
        
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
    
    def object_callback(self, msg):
        rospy.loginfo("Received object pose, starting pick and place operation")
        
        # Move to home position first
        self.move_to_home()
        
        # Open gripper
        self.open_gripper()
        
        # Pre-grasp pose (slightly above the object)
        pre_grasp_pose = geometry_msgs.msg.Pose()
        pre_grasp_pose.position.x = msg.pose.position.x
        pre_grasp_pose.position.y = msg.pose.position.y
        pre_grasp_pose.position.z = msg.pose.position.z + 0.1  # 10cm above object
        
        # Set orientation for top-down grasp
        q = quaternion_from_euler(0, np.pi/2, 0)  # Roll, Pitch, Yaw
        pre_grasp_pose.orientation.x = q[0]
        pre_grasp_pose.orientation.y = q[1]
        pre_grasp_pose.orientation.z = q[2]
        pre_grasp_pose.orientation.w = q[3]
        
        # Move to pre-grasp pose
        self.move_group.set_pose_target(pre_grasp_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if not success:
            rospy.logwarn("Failed to reach pre-grasp pose")
            return
        
        # Approach object
        grasp_pose = geometry_msgs.msg.Pose()
        grasp_pose.position.x = msg.pose.position.x
        grasp_pose.position.y = msg.pose.position.y
        grasp_pose.position.z = msg.pose.position.z + 0.02  # Slight offset to avoid collision
        grasp_pose.orientation = pre_grasp_pose.orientation
        
        # Use Cartesian path for approach
        waypoints = []
        waypoints.append(grasp_pose)
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0          # jump_threshold
        )
        
        if fraction < 0.9:
            rospy.logwarn(f"Only achieved {fraction:.2f} of approach path")
        
        self.move_group.execute(plan, wait=True)
        
        # Close gripper to grasp object
        self.close_gripper()
        rospy.sleep(0.5)  # Wait for gripper to close
        
        # Move to place position
        place_pose = geometry_msgs.msg.Pose()
        place_pose.position.x = 0.2
        place_pose.position.y = -0.2  # 20cm to the left
        place_pose.position.z = 0.1
        place_pose.orientation = pre_grasp_pose.orientation
        
        self.move_group.set_pose_target(place_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if not success:
            rospy.logwarn("Failed to reach place pose")
            # Try to recover by moving home
            self.move_to_home()
            return
        
        # Open gripper to release object
        self.open_gripper()
        rospy.sleep(0.5)  # Wait for gripper to open
        
        # Move back to home position
        self.move_to_home()
        
        rospy.loginfo("Pick and place operation completed")

if __name__ == '__main__':
    try:
        pick_place = PickAndPlace()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

### 4.2 Launch File for Pick-and-Place

```xml
<launch>
    <!-- Start MoveIt -->
    <include file="$(find px100_moveit)/launch/px100_moveit.launch">
        <arg name="use_actual" value="true"/>
    </include>
    
    <!-- Start perception pipeline -->
    <include file="$(find kinetirover_perception)/launch/perception.launch"/>
    
    <!-- Start pick and place node -->
    <node name="pick_and_place" pkg="kinetirover_manipulation" type="pick_and_place.py" output="screen"/>
</launch>
```

## 5. Calibration and Testing

### 5.1 Camera-Robot Calibration

For accurate pick and place, the camera needs to be properly calibrated with the robot's coordinate system:

1. Use an ArUco marker on the robot's gripper
2. Move the robot to several known positions
3. Calculate the transform between camera and robot base frames

### 5.2 Testing the Pipeline

To test the perception pipeline:

1. Launch the perception system:
   ```bash
   roslaunch kinetirover_perception perception.launch
   ```

2. Place objects in the robot's workspace

3. Visualize the detection results in RViz:
   - Add a Camera display for the RGB image
   - Add a PointCloud2 display for the filtered points
   - Add a MarkerArray display for the detected objects

4. Launch the pick-and-place demo:
   ```bash
   roslaunch kinetirover_manipulation pick_place_demo.launch
   ```

## 6. Troubleshooting

### Common issues and solutions:

1. **Objects not detected**:
   - Check that the YOLO model is properly trained for your objects
   - Ensure proper lighting conditions
   - Verify camera is properly positioned

2. **Inaccurate object positions**:
   - Recalibrate the camera-robot transform
   - Check for reflective surfaces that may affect depth accuracy
   - Adjust the point cloud filtering parameters

3. **Robot fails to grasp objects**:
   - Ensure gripper size is appropriate for the objects
   - Check that object positions are within the robot's workspace
   - Adjust the grasp height offset

## 7. Advanced Topics

### 7.1 Training Custom YOLO Models

To detect specific objects for your application, you may need to train a custom YOLO model:

1. Collect images of your objects (at least 100 per class)
2. Annotate the images with bounding boxes
3. Train the model using Darknet
4. Convert the model to use with darknet_ros

### 7.2 Improving Perception Robustness

- Use temporal filtering to reduce noise in object positions
- Implement multi-view fusion for better 3D position estimation
- Add object tracking to maintain persistent object IDs

## Conclusion

In this tutorial, we've set up a complete perception pipeline for the KinetiRover, enabling it to:
1. Process point clouds from the RealSense camera
2. Detect objects using YOLO
3. Estimate 3D positions of objects
4. Perform pick-and-place operations with MoveIt

This perception system can be extended for various applications such as sorting objects, assembly tasks, or collaborative robot operations.

---
*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*