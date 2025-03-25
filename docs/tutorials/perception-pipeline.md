# Perception Pipeline Setup

*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*

## Overview

This tutorial will guide you through setting up a complete perception pipeline for the PX100 robot using the Intel RealSense D435 camera and darknet_ros_3d for object detection. The perception system will detect objects in 3D space and provide their position and bounding box information to the manipulation system.

## Prerequisites

- ROS Noetic installed
- PX100 robot setup completed
- RealSense D435 camera installed and configured
- Basic knowledge of Python and ROS

## Dependencies Installation

1. Install the required packages:
```bash
sudo apt-get install ros-noetic-cv-bridge ros-noetic-vision-opencv python3-opencv
sudo apt-get install ros-noetic-darknet-ros
```

2. Clone the gb_visual_detection_3d repository:
```bash
cd ~/kinetibot_ws/src
git clone https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git
```

3. Build the workspace:
```bash
cd ~/kinetibot_ws
catkin_make
source devel/setup.bash
```

## Creating the Perception Package

1. Create a new package for the perception pipeline:
```bash
cd ~/kinetibot_ws/src
catkin_create_pkg px100_perception std_msgs sensor_msgs darknet_ros_3d_msgs darknet_ros_msgs rospy pcl_ros
```

2. Create the necessary directories:
```bash
cd px100_perception
mkdir -p launch config scripts
```

## Object Detection Setup

1. Create a Python script for object detection processing in `scripts/object_detector.py`:

```python
#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point
import sensor_msgs.point_cloud2 as pc2

class PX100ObjectDetector:
    def __init__(self):
        rospy.init_node('px100_object_detector', anonymous=True)
        
        # Parameters
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.base_frame = rospy.get_param('~base_frame', 'px100/base_link')
        self.min_probability = rospy.get_param('~min_probability', 0.5)
        self.target_objects = rospy.get_param('~target_objects', ['bottle', 'cup', 'box'])
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Subscribe to topics
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.pc_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)
        self.darknet_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.darknet_callback)
        
        # Publishers
        self.bb3d_pub = rospy.Publisher('/px100/detected_objects', BoundingBoxes3d, queue_size=1)
        self.detections_pub = rospy.Publisher('/px100/detection_visualization', Image, queue_size=1)
        
        # Class variables
        self.current_pc = None
        self.current_depth = None
        self.current_boxes = None
        self.latest_boxes3d = BoundingBoxes3d()
        
        rospy.loginfo('PX100 Object Detector initialized')
    
    def depth_callback(self, msg):
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    
    def pointcloud_callback(self, msg):
        self.current_pc = msg
        # Process detections if we have all required data
        if self.current_boxes is not None and self.current_depth is not None:
            self.process_detections()
    
    def darknet_callback(self, msg):
        self.current_boxes = msg
    
    def process_detections(self):
        if self.current_boxes is None or self.current_pc is None:
            return
            
        # Create a new BoundingBoxes3d message
        boxes3d = BoundingBoxes3d()
        boxes3d.header.stamp = rospy.Time.now()
        boxes3d.header.frame_id = self.base_frame
        
        # Process each bounding box
        for box in self.current_boxes.bounding_boxes:
            # Skip if probability is too low or not in target objects
            if box.probability < self.min_probability or box.Class not in self.target_objects:
                continue
                
            # Calculate center point of bounding box in image coordinates
            center_x = int((box.xmin + box.xmax) / 2)
            center_y = int((box.ymin + box.ymax) / 2)
            
            # Get point cloud data for the center point
            # Extract 3D position from point cloud
            point_3d = self.get_3d_point(center_x, center_y)
            if point_3d is None:
                continue
                
            # Create 3D bounding box
            bb3d = BoundingBox3d()
            bb3d.Class = box.Class
            bb3d.probability = box.probability
            bb3d.xmin = point_3d.x - 0.1  # Approximate size
            bb3d.xmax = point_3d.x + 0.1
            bb3d.ymin = point_3d.y - 0.1
            bb3d.ymax = point_3d.y + 0.1
            bb3d.zmin = point_3d.z - 0.1
            bb3d.zmax = point_3d.z + 0.1
            bb3d.center = point_3d
            
            boxes3d.bounding_boxes.append(bb3d)
        
        # Publish 3D bounding boxes
        self.bb3d_pub.publish(boxes3d)
        self.latest_boxes3d = boxes3d
        
        # Create visualization
        self.create_visualization()
    
    def get_3d_point(self, x, y):
        try:
            # Create a generator of points in the point cloud
            pc_gen = pc2.read_points(self.current_pc, field_names=('x', 'y', 'z'), skip_nans=True, uvs=[(x, y)])
            
            # Get the first point
            for p in pc_gen:
                # Transform point to base frame
                point_stamped = PoseStamped()
                point_stamped.header.frame_id = self.camera_frame
                point_stamped.pose.position.x = p[0]
                point_stamped.pose.position.y = p[1]
                point_stamped.pose.position.z = p[2]
                point_stamped.pose.orientation.w = 1.0
                
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    rospy.Time(0),
                    rospy.Duration(1.0))
                
                transformed_point = tf2_geometry_msgs.do_transform_pose(point_stamped, transform)
                
                return transformed_point.pose.position
                
        except Exception as e:
            rospy.logerr(f"Error getting 3D point: {e}")
            return None
    
    def create_visualization(self):
        # TODO: Add visualization code using OpenCV
        pass
    
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = PX100ObjectDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
```

2. Make the script executable:
```bash
chmod +x scripts/object_detector.py
```

## Configuration

1. Create a configuration file for darknet_ros_3d in `config/px100_darknet_3d.yaml`:
```yaml
darknet_ros_3d:
  yolo_model:
    config_file: 
      name: /config/yolov3.cfg
    weight_file:
      name: /weights/yolov3.weights
    threshold:
      value: 0.5
    detection_classes:
      names:
        - bottle
        - cup
        - book
        - box
  pointcloud_processing:
    working_frame: px100/base_link
    min_probability: 0.4
    min_points: 50
    max_points: 1000
```

## Launch File Setup

1. Create a launch file for the perception pipeline in `launch/px100_perception.launch`:
```xml
<launch>
  <!-- Arguments -->
  <arg name="camera_frame" default="camera_color_optical_frame" />
  <arg name="base_frame" default="px100/base_link" />
  <arg name="min_probability" default="0.5" />
  <arg name="pointcloud_topic" default="/camera/depth/color/points" />
  
  <!-- RealSense Camera -->
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="align_depth" value="true" />
    <arg name="color_width" value="640" />
    <arg name="color_height" value="480" />
    <arg name="color_fps" value="30" />
    <arg name="depth_width" value="640" />
    <arg name="depth_height" value="480" />
    <arg name="depth_fps" value="30" />
  </include>
  
  <!-- Darknet ROS -->
  <include file="$(find darknet_ros)/launch/darknet_ros.launch">
    <arg name="image" value="/camera/color/image_raw" />
  </include>
  
  <!-- Darknet ROS 3D (converts 2D detections to 3D) -->
  <include file="$(find darknet_ros_3d)/launch/darknet_ros_3d.launch">
    <arg name="input_bbx_topic" value="/darknet_ros/bounding_boxes" />
    <arg name="pointcloud_topic" value="$(arg pointcloud_topic)" />
    <arg name="working_frame" value="$(arg base_frame)" />
    <arg name="mininum_probability" value="$(arg min_probability)" />
  </include>
  
  <!-- Custom Object Detector Node -->
  <node name="px100_object_detector" pkg="px100_perception" type="object_detector.py" output="screen">
    <param name="camera_frame" value="$(arg camera_frame)" />
    <param name="base_frame" value="$(arg base_frame)" />
    <param name="min_probability" value="$(arg min_probability)" />
    <rosparam param="target_objects">["bottle", "cup", "box"]</rosparam>
  </node>
  
  <!-- Visualization in RViz -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find px100_perception)/config/perception.rviz" />
</launch>
```

## Using the Detected Objects

The perception pipeline publishes detected objects as `BoundingBoxes3d` messages on the `/px100/detected_objects` topic. You can use these detections for manipulation tasks.

Here's an example of how to subscribe to and use the detections:

```python
#!/usr/bin/env python3

import rospy
from darknet_ros_3d_msgs.msg import BoundingBoxes3d

def detection_callback(msg):
    if len(msg.bounding_boxes) == 0:
        rospy.loginfo("No objects detected")
        return
        
    for box in msg.bounding_boxes:
        rospy.loginfo(f"Detected {box.Class} at position: "
                     f"({box.center.x:.3f}, {box.center.y:.3f}, {box.center.z:.3f}) "
                     f"with probability: {box.probability:.2f}")
        
        # You can now use this position for grasping or other manipulation tasks

if __name__ == '__main__':
    rospy.init_node('detection_subscriber')
    rospy.Subscriber('/px100/detected_objects', BoundingBoxes3d, detection_callback)
    rospy.spin()
```

## Running the Perception Pipeline

1. Start the perception pipeline:
```bash
roslaunch px100_perception px100_perception.launch
```

2. Place objects in view of the camera.

3. Check the detected objects:
```bash
rostopic echo /px100/detected_objects
```

## Troubleshooting

### Common Issues

1. **No detections**:
   - Ensure the camera is properly connected and streaming
   - Check camera field of view - objects should be clearly visible
   - Adjust the `min_probability` parameter if needed

2. **Inaccurate 3D positions**:
   - Verify camera calibration
   - Check TF tree for correct transformations
   - Ensure proper lighting conditions

3. **High CPU usage**:
   - Lower camera resolution in the launch file
   - Reduce the detection frequency

## Next Steps

- Implement a grasp generator based on detected objects
- Integrate the perception pipeline with MoveIt for pick and place operations
- Add custom object detection training for specific objects

---

*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*
