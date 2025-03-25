# Perception Pipeline

*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*

## Overview

This tutorial covers setting up the perception pipeline for the KinetiRover, which uses the RealSense D435 camera and darknet_ros_3d for object detection and localization. We'll be using the darknet_ros_3d_msgs for processing bounding boxes and integrating with the existing PickAndPlace class.

## Prerequisites

- Completed the [RealSense Setup](realsense-setup.md) tutorial
- ROS Noetic installed
- KinetiRover repository cloned and built
- darknet_ros_3d package installed

## Install Dependencies

```bash
# Install darknet_ros_3d
cd ~/kinetibot_ws/src
git clone https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
cd ..
catkin_make
source devel/setup.bash

# Install additional dependencies
sudo apt install ros-noetic-pcl-ros ros-noetic-pcl-conversions python3-pcl
```

## Perception Pipeline Components

Our perception system consists of the following components:

1. **RealSense D435** - Provides RGB and depth data
2. **darknet_ros_3d** - Performs object detection and 3D localization
3. **PCL processing** - Filters and segments point clouds
4. **PickAndPlace class** - Handles object manipulation based on perception data

## Perception Node Implementation

Let's create a Python node that subscribes to the darknet_ros_3d_msgs and integrates with our PickAndPlace class:

```python
#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point
from darknet_ros_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector_node')
        
        # Parameters
        self.target_frame = rospy.get_param('~target_frame', 'base_link')
        self.min_confidence = rospy.get_param('~min_confidence', 0.5)
        self.target_objects = rospy.get_param('~target_objects', ['bottle', 'cup'])
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers
        self.marker_pub = rospy.Publisher('detected_objects', MarkerArray, queue_size=1)
        
        # Subscribers
        rospy.Subscriber(
            'darknet_ros_3d/bounding_boxes', 
            BoundingBoxes3d, 
            self.detection_callback
        )
        
        # Initialize the PickAndPlace interface
        self.pick_place = PickAndPlace()
        
        rospy.loginfo("Object detector node initialized")
    
    def detection_callback(self, bbox_msg):
        # Create marker array for visualization
        marker_array = MarkerArray()
        detected_objects = []
        
        for i, bbox in enumerate(bbox_msg.bounding_boxes):
            # Skip objects with low confidence or not in target list
            if bbox.probability < self.min_confidence or bbox.Class not in self.target_objects:
                continue
            
            # Create pose from bounding box center
            object_pose = PoseStamped()
            object_pose.header = bbox_msg.header
            object_pose.pose.position = bbox.center
            object_pose.pose.orientation.w = 1.0
            
            # Transform to target frame if needed
            if bbox_msg.header.frame_id != self.target_frame:
                try:
                    object_pose = self.tf_buffer.transform(object_pose, self.target_frame)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(f"TF Error: {e}")
                    continue
            
            # Add to detected objects list
            detected_objects.append({
                'class': bbox.Class,
                'pose': object_pose.pose,
                'dimensions': {
                    'x': bbox.size.x,
                    'y': bbox.size.y,
                    'z': bbox.size.z
                },
                'probability': bbox.probability
            })
            
            # Create marker for visualization
            marker = Marker()
            marker.header = object_pose.header
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = object_pose.pose
            marker.scale = bbox.size
            marker.color.r = 1.0 if bbox.Class == 'bottle' else 0.0
            marker.color.g = 0.0 if bbox.Class == 'bottle' else 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            marker.lifetime = rospy.Duration(0.5)
            marker_array.markers.append(marker)
            
            # Add text marker for the class name
            text_marker = Marker()
            text_marker.header = object_pose.header
            text_marker.id = i + 1000  # Offset to avoid ID collision
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = object_pose.pose
            text_marker.pose.position.z += (bbox.size.z / 2) + 0.05  # Position text above object
            text_marker.text = f"{bbox.Class}: {bbox.probability:.2f}"
            text_marker.scale.z = 0.05  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime = rospy.Duration(0.5)
            marker_array.markers.append(text_marker)
        
        # Publish markers
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
            
        # Update PickAndPlace with detected objects
        if detected_objects:
            self.pick_place.update_objects(detected_objects)

# The PickAndPlace class is kept as is from the existing implementation
class PickAndPlace:
    def __init__(self):
        # Initialize connection to the robot
        rospy.loginfo("Initializing PickAndPlace interface")
        # Any initialization from the existing class...
        
    def update_objects(self, detected_objects):
        # Process detected objects for potential picking
        if not detected_objects:
            rospy.loginfo("No objects detected for picking")
            return
        
        # Sort objects by confidence
        sorted_objects = sorted(detected_objects, key=lambda obj: obj['probability'], reverse=True)
        rospy.loginfo(f"Found {len(sorted_objects)} objects for potential picking")
        
        # Further processing with the existing PickAndPlace class...
        # This would include planning grasps, executing pick and place movements, etc.

if __name__ == "__main__":
    try:
        detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

## Launch File

Create a launch file to run the perception pipeline:

```xml
<launch>
  <!-- Start RealSense camera -->
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="align_depth" value="true"/>
  </include>
  
  <!-- Start darknet_ros_3d -->
  <include file="$(find darknet_ros_3d)/launch/darknet_ros_3d.launch"/>
  
  <!-- Start our object detector node -->
  <node name="object_detector" pkg="kinetibot_perception" type="object_detector.py" output="screen">
    <param name="target_frame" value="base_link"/>
    <param name="min_confidence" value="0.60"/>
    <rosparam param="target_objects">["bottle", "cup", "book"]</rosparam>
  </node>
  
  <!-- Start RViz for visualization -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find kinetibot_perception)/config/perception.rviz"/>
</launch>
```

## RViz Configuration

Create an RViz configuration to visualize the perception pipeline:

1. Add the RGB and depth image displays for the RealSense camera
2. Add a PointCloud2 display for the processed point cloud
3. Add a MarkerArray display for the detected objects
4. Add a TF display to visualize coordinate frames

## Running the Perception Pipeline

```bash
# Start the perception pipeline
roslaunch kinetibot_perception perception_pipeline.launch
```

## Integrating with MoveIt

The PickAndPlace class will interact with MoveIt to perform actual manipulation based on the perception data. Key integration points include:

1. Converting detected object poses to the robot's base frame
2. Planning grasp approaches based on object dimensions
3. Executing pick and place movements

This integration will be covered in more detail in the [MoveIt Integration](moveit-setup.md) tutorial.

## Troubleshooting

### Common Issues

1. **No objects detected**
   - Check that the camera is properly connected
   - Verify darknet_ros_3d is running
   - Ensure proper lighting conditions

2. **Incorrect 3D positions**
   - Check camera calibration
   - Verify TF tree is correctly set up
   - Adjust the camera position for better view

3. **Low detection confidence**
   - Try retraining the detection model
   - Adjust lighting conditions
   - Ensure objects are clearly visible

### Verifying the Pipeline

You can check individual components with these commands:

```bash
# Check darknet_ros_3d detections
rostopic echo /darknet_ros_3d/bounding_boxes

# Visualize detected objects
rostopic echo /detected_objects

# Check camera point cloud
rostopic echo /camera/depth/color/points
```

## Next Steps

Now that you have a working perception pipeline, proceed to the [MoveIt Integration](moveit-setup.md) tutorial to learn how to use this perception data for manipulation tasks.

*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*