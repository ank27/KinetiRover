# Perception Pipeline: Object Detection and Pose Estimation

*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*

## Overview

This tutorial explains how to set up the perception pipeline for the KinetiRover project using RealSense D435 camera data along with darknet_ros and darknet_ros_3d for object detection and pose estimation.

## Prerequisites

- Complete the [RealSense Setup](realsense-setup.md) tutorial
- ROS Noetic installed
- CUDA and cuDNN installed (for optimal Darknet performance)

## Installation

### 1. Install darknet_ros

```bash
cd ~/kinetibot_ws/src
git clone https://github.com/leggedrobotics/darknet_ros.git
```

### 2. Install gb_visual_detection_3d and dependencies

```bash
cd ~/kinetibot_ws/src
git clone https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
```

### 3. Install any missing dependencies

```bash
cd ~/kinetibot_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the workspace

```bash
catkin_make
source devel/setup.bash
```

## Perception Pipeline Setup

### 1. Create a perception package

We'll create a package specifically for the KinetiRover perception pipeline:

```bash
cd ~/kinetibot_ws/src
catkin_create_pkg px100_perception std_msgs rospy sensor_msgs darknet_ros_msgs darknet_ros_3d_msgs
```

### 2. Set up the perception node

Create a new Python script for object detection and pose estimation:

```bash
mkdir -p ~/kinetibot_ws/src/px100_perception/scripts
touch ~/kinetibot_ws/src/px100_perception/scripts/px100_object_detector.py
chmod +x ~/kinetibot_ws/src/px100_perception/scripts/px100_object_detector.py
```

Edit the file with your favorite editor and add the following code:

```python
#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from geometry_msgs.msg import PoseStamped, TransformStamped

class PX100ObjectDetector:
    def __init__(self):
        rospy.init_node('px100_object_detector')
        
        # Publishers
        self.detected_objects_pub = rospy.Publisher('/px100/detected_objects', BoundingBoxes3d, queue_size=10)
        self.target_pose_pub = rospy.Publisher('/px100/target_pose', PoseStamped, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d, self.bbox_callback)
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("PX100 Object Detector node initialized")
        
    def bbox_callback(self, msg):
        # Forward the bounding boxes to our topic
        self.detected_objects_pub.publish(msg)
        
        # Find objects of interest (e.g., "cube", "bottle", etc.)
        for bbox in msg.bounding_boxes:
            if bbox.Class in ['cube', 'bottle', 'can']:
                rospy.loginfo(f"Found object: {bbox.Class}")
                
                # Create pose from bounding box center
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = bbox.xmin + (bbox.xmax - bbox.xmin) / 2.0
                pose.pose.position.y = bbox.ymin + (bbox.ymax - bbox.ymin) / 2.0
                pose.pose.position.z = bbox.zmin + (bbox.zmax - bbox.zmin) / 2.0
                pose.pose.orientation.w = 1.0  # Default orientation
                
                # Transform pose to base_link frame if needed
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'px100/base_link',
                        msg.header.frame_id,
                        rospy.Time(0),
                        rospy.Duration(1.0))
                    transformed_pose = tf2_geometry_msgs.do_transform_pose(
                        pose, transform)
                    self.target_pose_pub.publish(transformed_pose)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                        tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(f"TF error: {e}")
                    # Publish untransformed pose as fallback
                    self.target_pose_pub.publish(pose)
                break  # Process only the first object of interest

if __name__ == '__main__':
    detector = PX100ObjectDetector()
    rospy.spin()
```

### 3. Create a launch file

Create a launch file to start the perception pipeline:

```bash
mkdir -p ~/kinetibot_ws/src/px100_perception/launch
touch ~/kinetibot_ws/src/px100_perception/launch/px100_perception.launch
```

Add the following content to the launch file:

```xml
<launch>
  <!-- Arguments -->
  <arg name="camera_name" default="camera"/>
  <arg name="rgb_topic" default="/$(arg camera_name)/color/image_raw"/>
  <arg name="depth_topic" default="/$(arg camera_name)/aligned_depth_to_color/image_raw"/>
  <arg name="camera_info" default="/$(arg camera_name)/color/camera_info"/>
  <arg name="point_cloud_topic" default="/$(arg camera_name)/depth/color/points"/>

  <!-- Launch RealSense camera -->
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="align_depth" value="true"/>
  </include>

  <!-- Launch Darknet ROS -->
  <include file="$(find darknet_ros)/launch/darknet_ros.launch">
    <arg name="image" value="$(arg rgb_topic)"/>
  </include>

  <!-- Launch Darknet ROS 3D -->
  <include file="$(find darknet_ros_3d)/launch/darknet_ros_3d.launch">
    <arg name="input_bbx_topic" value="/darknet_ros/bounding_boxes"/>
    <arg name="input_cloud_topic" value="$(arg point_cloud_topic)"/>
    <arg name="output_bbx3d_topic" value="/darknet_ros_3d/bounding_boxes"/>
    <arg name="working_frame" value="camera_link"/>
    <arg name="minimum_probability" value="0.3"/>
  </include>

  <!-- Launch our perception node -->
  <node pkg="px100_perception" type="px100_object_detector.py" name="px100_object_detector" output="screen"/>
</launch>
```

### 4. Configure package.xml

Update the package.xml file to include the correct dependencies:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>px100_perception</name>
  <version>0.0.1</version>
  <description>Perception pipeline for KinetiRover using PX100</description>

  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <depend>rospy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>darknet_ros_msgs</depend>
  <depend>darknet_ros_3d_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <exec_depend>darknet_ros</exec_depend>
  <exec_depend>darknet_ros_3d</exec_depend>
  <exec_depend>realsense2_camera</exec_depend>

  <export>
  </export>
</package>
```

## Running the Perception Pipeline

```bash
# Make sure your workspace is built and sourced
cd ~/kinetibot_ws
catkin_make
source devel/setup.bash

# Launch the perception pipeline
roslaunch px100_perception px100_perception.launch
```

## Visualization

To visualize the detected objects, use RViz:

```bash
rosrun rviz rviz
```

Add these displays:
1. Image (topic: /darknet_ros/detection_image)
2. PointCloud2 (topic: /camera/depth/color/points)
3. MarkerArray (topic: /darknet_ros_3d/markers)

## Testing Object Detection

1. Place objects in front of the RealSense camera
2. Check the terminal for detection messages
3. Verify bounding boxes in RViz
4. Check the pose data:
   ```bash
   rostopic echo /px100/target_pose
   ```

## Next Steps

Now that you have a working perception system that can detect objects and estimate their 3D poses, you're ready to move on to [MoveIt Integration](moveit-setup.md) to implement pick and place functionality.

---

*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*
