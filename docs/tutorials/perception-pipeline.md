# Perception Pipeline Setup

*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*

## Overview

The KinetiRover perception pipeline integrates the RealSense D435 camera with object detection algorithms to enable the robot to recognize and interact with objects in its environment. This guide will walk you through setting up the complete perception system.

## Prerequisites

Before setting up the perception pipeline, ensure you have:
- Completed the [RealSense Setup](realsense-setup.md) tutorial
- ROS Noetic installed with the following packages:
  - PCL (Point Cloud Library)
  - OpenCV
  - gb_visual_detection packages

## Installation Steps

### 1. Install Darknet ROS 3D

```bash
# Install dependencies
sudo apt-get install ros-noetic-pcl-ros ros-noetic-perception-pcl

# Clone the gb_visual_detection packages
cd ~/kinetibot_ws/src
git clone https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
cd ..

# Build the workspace
catkin_make
source devel/setup.bash
```

### 2. Configure Darknet ROS 3D

The perception system uses darknet_ros_3d_msgs for BoundingBoxArray and BoundingBox detection. These messages provide standardized formats for object detection results.

```bash
# Install YOLOv4 weights (if not already installed)
cd ~/kinetibot_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
```

## Perception Pipeline Architecture

The KinetiRover perception pipeline consists of several nodes:

1. **RealSense Camera Node**: Provides RGB and depth data
2. **Point Cloud Generation**: Converts depth data to 3D point clouds
3. **Object Detection**: Uses YOLOv4 for 2D object detection
4. **3D Mapping**: Maps 2D detections to 3D space using point clouds
5. **Object Tracking**: Maintains consistent object IDs across frames

## Launch Files

### Basic Perception Pipeline

```bash
# Launch the perception pipeline
roslaunch px100_perception kinetibot_perception.launch
```

### Advanced Configuration with Custom Objects

```bash
# Launch with custom object detection
roslaunch px100_perception kinetibot_perception.launch custom_objects:=true
```

## Working with BoundingBoxArray and BoundingBox Messages

The perception pipeline publishes detected objects using the `darknet_ros_3d_msgs/BoundingBoxArray` message type. Here's how to work with these messages:

### Message Structure

```
# BoundingBox message structure
string class_id      # Object class name
float64 probability  # Detection confidence
float64 xmin         # 3D bounding box min X
float64 ymin         # 3D bounding box min Y
float64 zmin         # 3D bounding box min Z
float64 xmax         # 3D bounding box max X
float64 ymax         # 3D bounding box max Y
float64 zmax         # 3D bounding box max Z

# BoundingBoxArray contains multiple BoundingBox objects
Header header
darknet_ros_3d_msgs/BoundingBox[] bounding_boxes
```

### Subscribing to Detection Messages

```python
#!/usr/bin/env python3
import rospy
from darknet_ros_3d_msgs.msg import BoundingBoxArray

def detection_callback(msg):
    for box in msg.bounding_boxes:
        rospy.loginfo("Detected object: %s with confidence %.2f%%", 
                     box.class_id, box.probability * 100)
        rospy.loginfo("3D position: Center at (%.2f, %.2f, %.2f)", 
                     (box.xmin + box.xmax) / 2,
                     (box.ymin + box.ymax) / 2,
                     (box.zmin + box.zmax) / 2)

if __name__ == '__main__':
    rospy.init_node('object_detection_subscriber')
    rospy.Subscriber('/darknet_ros_3d/bounding_boxes', 
                     BoundingBoxArray, detection_callback)
    rospy.spin()
```

## Tuning the Perception Pipeline

### Point Cloud Filtering

You can adjust the point cloud filtering parameters in the configuration file:

```yaml
# Example configuration for point cloud filtering
filter:
  voxel_size: 0.01  # Voxel grid filter size
  z_limit_min: 0.1  # Minimum Z-axis limit (meters)
  z_limit_max: 1.5  # Maximum Z-axis limit (meters)
```

### Object Detection Confidence

Adjust the detection confidence threshold in the configuration file:

```yaml
# Detection confidence threshold
detection:
  threshold: 0.3  # Minimum confidence (0-1)
```

## Visualizing Detection Results

To visualize the detected objects in RViz:

1. Launch RViz:
   ```bash
   rosrun rviz rviz
   ```

2. Add the following displays:
   - PointCloud2 (topic: `/camera/depth/color/points`)
   - MarkerArray (topic: `/darknet_ros_3d/markers`)

3. Set the fixed frame to `camera_link` or whichever frame your camera is using

## Adding Custom Object Detection

To train the system to recognize new objects:

1. Prepare your training dataset
2. Update the object classes file at `config/obj.names`
3. Train a new YOLOv4 model (or use a pre-trained one)
4. Update the weights file in the launch configuration

## Troubleshooting

### Common Issues

1. **No objects detected**:
   - Check camera positioning and lighting
   - Verify the confidence threshold isn't too high
   - Ensure objects are within the detection range

2. **Inaccurate 3D positions**:
   - Calibrate the RealSense camera
   - Adjust the point cloud filtering parameters
   - Check for reflective or transparent surfaces

3. **System performance issues**:
   - Lower the camera resolution or frame rate
   - Reduce the point cloud density
   - Consider using a more powerful GPU

## Next Steps

Now that you have set up the perception pipeline, you can proceed to the [MoveIt Integration](moveit-setup.md) tutorial to learn how to use the detected object information for planning and manipulation tasks.

---
*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*