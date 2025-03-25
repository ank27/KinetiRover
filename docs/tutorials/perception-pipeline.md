# Perception Pipeline with Darknet and RealSense D435

*Previous: [RealSense Integration](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*

## Overview

This tutorial covers setting up the perception pipeline for the PX100 robot using Darknet and Darknet3D with the RealSense D435 camera. The system uses YOLO-based object detection combined with depth information to locate objects in 3D space.

## Prerequisites

- Complete the [RealSense Integration](realsense-setup.md) tutorial
- Ubuntu 20.04 with ROS Noetic installed
- CUDA support (recommended for better performance)
- RealSense D435 camera properly connected and configured

## Dependencies Installation

1. Install required packages:

```bash
sudo apt-get update
sudo apt-get install ros-noetic-darknet-ros
sudo apt-get install python3-pip
pip3 install numpy opencv-python
```

2. Clone the gb_visual_detection repository for darknet_ros_3d:

```bash
cd ~/kinetibot_ws/src
git clone https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
```

3. Install additional dependencies and build the workspace:

```bash
cd ~/kinetibot_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Perception Package Setup

1. Create the perception package:

```bash
cd ~/kinetibot_ws/src
catkin_create_pkg px100_perception std_msgs sensor_msgs darknet_ros_msgs darknet_ros_3d_msgs roscpp rospy
cd px100_perception
mkdir -p launch config scripts
```

2. Create the Python Node for Object Detection

Create a new file `scripts/px100_object_detection.py`:

```bash
touch ~/kinetibot_ws/src/px100_perception/scripts/px100_object_detection.py
chmod +x ~/kinetibot_ws/src/px100_perception/scripts/px100_object_detection.py
```

Add the following code to the file:

```python
#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_3d_msgs.msg import BoundingBox3d, BoundingBoxes3d
import sensor_msgs.point_cloud2 as pc2

class PX100ObjectDetection:
    def __init__(self):
        rospy.init_node('px100_object_detection', anonymous=True)
        
        # Parameters
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.target_frame = rospy.get_param('~target_frame', 'base_link')
        self.min_probability = rospy.get_param('~min_probability', 0.5)
        self.min_points = rospy.get_param('~min_points', 10)
        
        # Subscribers
        self.bbox_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', 
                                        BoundingBoxes, self.bbox_callback)
        self.pc_sub = rospy.Subscriber('/camera/depth/color/points', 
                                      PointCloud2, self.pc_callback)
        
        # Publishers
        self.bbox3d_pub = rospy.Publisher('/px100/detected_objects', 
                                        BoundingBoxes3d, queue_size=1)
        
        # Initialize variables
        self.latest_pc = None
        self.latest_bboxes = None
        
        rospy.loginfo("PX100 Object Detection node initialized")
        
    def pc_callback(self, pc_msg):
        self.latest_pc = pc_msg
        if self.latest_bboxes is not None:
            self.process_detections()
    
    def bbox_callback(self, bbox_msg):
        self.latest_bboxes = bbox_msg
        if self.latest_pc is not None:
            self.process_detections()
    
    def process_detections(self):
        if self.latest_pc is None or self.latest_bboxes is None:
            return
            
        pc_array = pc2.read_points(self.latest_pc, skip_nans=True, field_names=("x", "y", "z"))
        pc_list = list(pc_array)
        
        if len(pc_list) == 0:
            return
            
        pc_array = np.array(pc_list)
        
        # Process each 2D bounding box
        bboxes3d_msg = BoundingBoxes3d()
        bboxes3d_msg.header = self.latest_pc.header
        
        for bbox in self.latest_bboxes.bounding_boxes:
            if bbox.probability < self.min_probability:
                continue
                
            # Extract points in the 2D bounding box
            points_in_bbox = []
            pc_msg = self.latest_pc
            
            # Convert 2D pixel coordinates to 3D points
            for point in pc2.read_points(pc_msg, skip_nans=True, field_names=("x", "y", "z", "rgb")):
                x, y, z = point[0:3]
                # Project 3D point to image plane (simplified - actual implementation would need camera intrinsics)
                # This is a placeholder - real implementation needs proper projection
                if x > 0 and z > 0:  # Points in front of camera
                    points_in_bbox.append((x, y, z))
            
            if len(points_in_bbox) < self.min_points:
                continue
                
            # Calculate 3D bounding box from points
            points = np.array(points_in_bbox)
            min_point = np.min(points, axis=0)
            max_point = np.max(points, axis=0)
            center = (min_point + max_point) / 2
            dimensions = max_point - min_point
            
            # Create 3D bounding box message
            bbox3d = BoundingBox3d()
            bbox3d.Class = bbox.Class
            bbox3d.probability = bbox.probability
            bbox3d.xmin = min_point[0]
            bbox3d.ymin = min_point[1]
            bbox3d.zmin = min_point[2]
            bbox3d.xmax = max_point[0]
            bbox3d.ymax = max_point[1]
            bbox3d.zmax = max_point[2]
            bbox3d.center.x = center[0]
            bbox3d.center.y = center[1]
            bbox3d.center.z = center[2]
            
            bboxes3d_msg.boxes.append(bbox3d)
        
        if len(bboxes3d_msg.boxes) > 0:
            self.bbox3d_pub.publish(bboxes3d_msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = PX100ObjectDetection()
        detector.run()
    except rospy.ROSInterruptException:
        pass
```

## Create Launch Files

1. Create a launch file for darknet_ros configuration:

```bash
touch ~/kinetibot_ws/src/px100_perception/launch/px100_darknet.launch
```

Add the following content:

```xml
<launch>
    <!-- YOLO Detection-->
    <include file="$(find darknet_ros)/launch/darknet_ros.launch">
        <arg name="config_path" value="$(find px100_perception)/config"/>
        <arg name="image" value="/camera/color/image_raw"/>
    </include>
</launch>
```

2. Create a launch file for the perception pipeline:

```bash
touch ~/kinetibot_ws/src/px100_perception/launch/px100_perception_pipeline.launch
```

Add the following content:

```xml
<launch>
    <!-- RealSense Camera -->
    <include file="$(find realsense2_camera)/launch/rs_camera.launch">
        <arg name="align_depth" value="true" />
        <arg name="filters" value="pointcloud" />
    </include>
    
    <!-- Darknet ROS -->
    <include file="$(find px100_perception)/launch/px100_darknet.launch" />
    
    <!-- 3D Object Detection Node -->
    <node pkg="px100_perception" type="px100_object_detection.py" name="px100_object_detection" output="screen">
        <param name="camera_frame" value="camera_color_optical_frame" />
        <param name="target_frame" value="base_link" />
        <param name="min_probability" value="0.5" />
        <param name="min_points" value="10" />
    </node>
</launch>
```

## Create Configuration Files

1. Create YOLO configuration folder:

```bash
mkdir -p ~/kinetibot_ws/src/px100_perception/config/yolo
```

2. Create the object classes file:

```bash
touch ~/kinetibot_ws/src/px100_perception/config/yolo/px100_classes.names
```

Add objects you want to detect, for example:

```
cube
cylinder
sphere
bot
```

3. Create YOLO configuration file:

```bash
touch ~/kinetibot_ws/src/px100_perception/config/yolo/px100.cfg
```

This file should have your YOLO model configuration. You can start with YOLOv3 or YOLOv4 configurations.

4. Create darknet_ros configuration file:

```bash
touch ~/kinetibot_ws/src/px100_perception/config/px100_darknet_ros.yaml
```

Add the following content:

```yaml
yolo_model:

  config_file:
    name: px100.cfg
  weight_file:
    name: px100.weights
  threshold:
    value: 0.5
  detection_classes:
    names:
      - cube
      - cylinder
      - sphere
      - bot
```

## Testing the Perception Pipeline

1. Build the workspace:

```bash
cd ~/kinetibot_ws
catkin_make
source devel/setup.bash
```

2. Launch the perception pipeline:

```bash
roslaunch px100_perception px100_perception_pipeline.launch
```

3. Visualize the results in RViz:

```bash
rosrun rviz rviz
```

In RViz, add the following displays:
1. PointCloud2 for the camera data
2. MarkerArray for the 3D bounding boxes

## Custom Object Training

To train your own custom model:

1. Collect and annotate images of objects you want to detect
2. Train YOLO using Darknet framework
3. Place the trained weights file in the `config/yolo` directory
4. Update the configuration files accordingly

## Integration with Robot Control

To integrate object detection with robot control, the 3D bounding boxes can be used for:

- Pick and place operations
- Obstacle avoidance
- Visual servoing
- Scene understanding

The detected objects are published on the `/px100/detected_objects` topic which can be subscribed by your robot control node.

## Troubleshooting

1. **No detections**:
   - Ensure camera is properly connected
   - Check lighting conditions
   - Verify the model is correctly loaded

2. **Inaccurate 3D bounding boxes**:
   - Camera might need calibration
   - Adjust the `min_points` parameter
   - Use a different point cloud filtering method

3. **Slow performance**:
   - Consider using a GPU
   - Reduce the input image resolution
   - Adjust the detection frequency

---

*Previous: [RealSense Integration](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*