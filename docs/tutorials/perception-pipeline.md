# Perception Pipeline for KinetiRover

*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*

## Introduction

The perception pipeline is a critical component of the KinetiRover robotic system. It bridges the gap between sensing the environment and taking action. In this tutorial, we'll implement a robust perception pipeline using Darknet (YOLO) and 3D object detection to identify objects in both 2D images and 3D space, then integrate with MoveIt for pick and place operations.

## Prerequisites

- Ubuntu 20.04 with ROS Noetic installed
- KinetiRover base setup completed
- RealSense D435 camera configured
- Basic understanding of Python programming

## 1. Perception System Architecture

Our perception pipeline consists of the following components:

```
+----------------+     +---------------+     +----------------+
| RealSense D435 | --> | Darknet YOLO  | --> | 3D Object      |
| Camera         |     | 2D Detection  |     | Localization   |
+----------------+     +---------------+     +----------------+
                                                     |
+----------------+     +---------------+     +----------------+
| MoveIt         | <-- | Pick & Place  | <-- | Object Pose    |
| Execution      |     | Planning      |     | Estimation     |
+----------------+     +---------------+     +----------------+
```

## 2. Installing Required Packages

### 2.1 Installing Darknet ROS

```bash
# Clone darknet_ros repository
cd ~/kinetibot_ws/src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git

# Build the workspace
cd ~/kinetibot_ws
catkin_make
```

### 2.2 Installing 3D Detection Packages

```bash
# Clone the gb_visual_detection_3d package
cd ~/kinetibot_ws/src
git clone https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d.git

# Install dependencies
sudo apt-get install ros-noetic-pcl-ros ros-noetic-vision-msgs

# Build the workspace again
cd ~/kinetibot_ws
catkin_make
```

## 3. Configuring the Perception Pipeline

### 3.1 Create a Launch File for Object Detection

Create a new launch file `perception_pipeline.launch` in your package:

```xml
<launch>
  <!-- Start RealSense camera -->
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="align_depth" value="true"/>
    <arg name="color_width" value="640"/>
    <arg name="color_height" value="480"/>
    <arg name="depth_width" value="640"/>
    <arg name="depth_height" value="480"/>
    <arg name="filters" value="pointcloud"/>
  </include>

  <!-- Start Darknet ROS YOLO -->
  <include file="$(find darknet_ros)/launch/yolo_v4.launch">
    <arg name="image" value="/camera/color/image_raw"/>
    <arg name="yolo_model_path" value="$(find darknet_ros)/yolo_network_config/weights"/>
    <arg name="yolo_config_path" value="$(find darknet_ros)/yolo_network_config/cfg"/>
  </include>

  <!-- Start 3D Object Detection -->
  <node name="darknet3d" pkg="darknet_ros_3d" type="darknet_ros_3d_node" output="screen">
    <param name="frame_id" value="camera_link"/>
    <param name="minimum_probability" value="0.4"/>
    <param name="minimum_detection_threshold" value="0.3"/>
    <param name="interested_classes" value="bottle,cup"/>
    <remap from="darknet_ros_3d/bounding_boxes" to="/darknet_ros_3d/bounding_boxes"/>
    <remap from="darknet_ros/bounding_boxes" to="/darknet_ros/bounding_boxes"/>
    <remap from="points" to="/camera/depth/color/points"/>
  </node>
</launch>
```

### 3.2 Understanding the 3D Object Detection

The 3D object detection node takes the 2D bounding boxes from YOLO detection and the point cloud from the RealSense camera to compute 3D bounding boxes. It outputs:

1. `gb_visual_detection_3d_msgs/BoundingBoxes3d` - 3D bounding boxes for detected objects
2. Visualization markers for RViz

## 4. Creating a Pick and Place Node with MoveIt Integration

Let's create a Python node to handle pick and place operations based on the 3D detections:

```python
#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class PickAndPlace:
    def __init__(self):
        rospy.init_node('darknet3d_pick_place', anonymous=True)
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Get robot and scene instances
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Setup arm and gripper group commanders
        self.arm_group = moveit_commander.MoveGroupCommander("px100_arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("px100_gripper")
        
        # Set planning parameters
        self.arm_group.set_planning_time(5)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_goal_position_tolerance(0.01)
        self.arm_group.set_goal_orientation_tolerance(0.05)
        
        # Initialize parameters
        self.target_object = rospy.get_param('~target_object', 'bottle')
        self.place_position = rospy.get_param('~place_position', [0.2, 0.2, 0.1])
        
        # Initialize TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Add a delay to allow initialization
        rospy.sleep(2)
        
        # Add a table to the scene
        self.add_table()
        
        # Subscribe to 3D bounding box topic
        self.detection_sub = rospy.Subscriber('/darknet_ros_3d/bounding_boxes', 
                                             BoundingBoxes3d, 
                                             self.detection_callback)
        
        rospy.loginfo("Pick and place node initialized. Waiting for object detections...")
        
    def add_table(self):
        # Add table as a collision object
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0.2
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -0.025  # Half of the table height
        table_pose.pose.orientation.w = 1.0
        
        # Add the box
        self.scene.add_box("table", table_pose, size=(0.5, 0.5, 0.05))
        rospy.sleep(1)  # Wait for the scene to update
        
    def open_gripper(self):
        # Open the gripper to its max position
        gripper_joint_goal = self.gripper_group.get_current_joint_values()
        gripper_joint_goal[0] = 0.037  # Open position for left_finger
        self.gripper_group.go(gripper_joint_goal, wait=True)
        self.gripper_group.stop()
        
    def close_gripper(self):
        # Close the gripper to grip an object
        gripper_joint_goal = self.gripper_group.get_current_joint_values()
        gripper_joint_goal[0] = 0.015  # Closed position for left_finger
        self.gripper_group.go(gripper_joint_goal, wait=True)
        self.gripper_group.stop()
        
    def move_to_home(self):
        # Move the arm to the home position
        joint_goal = self.arm_group.get_current_joint_values()
        joint_goal[0] = 0  # waist
        joint_goal[1] = 0  # shoulder
        joint_goal[2] = 0  # elbow
        joint_goal[3] = 0  # wrist
        
        self.arm_group.go(joint_goal, wait=True)
        self.arm_group.stop()
        
    def detection_callback(self, msg):
        # Skip if no detections
        if len(msg.bounding_boxes) == 0:
            return
        
        # Find our target object
        target_box = None
        for box in msg.bounding_boxes:
            if box.class_id == self.target_object:
                target_box = box
                break
        
        if target_box is None:
            return
        
        rospy.loginfo(f"Found target object '{self.target_object}'. Starting pick and place operation.")
        
        # Convert the box pose to gripper pose for picking
        grasp_pose = self.calculate_grasp_pose(target_box)
        
        # Execute pick and place
        self.execute_pick_place(grasp_pose)
        
    def calculate_grasp_pose(self, box):
        # Create a pose for grasping the object
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = box.header.frame_id
        grasp_pose.header.stamp = rospy.Time.now()
        
        # Position at the center of the box
        grasp_pose.pose.position.x = box.center.x
        grasp_pose.pose.position.y = box.center.y
        grasp_pose.pose.position.z = box.center.z + box.size.z/2 + 0.05  # Add offset for approach
        
        # Orientation for top-down grasp
        q = quaternion_from_euler(0, pi/2, 0)  # Roll, Pitch, Yaw
        grasp_pose.pose.orientation.x = q[0]
        grasp_pose.pose.orientation.y = q[1]
        grasp_pose.pose.orientation.z = q[2]
        grasp_pose.pose.orientation.w = q[3]
        
        # Transform to base_link frame if needed
        if grasp_pose.header.frame_id != "base_link":
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    grasp_pose.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
                grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, transform)
                grasp_pose.header.frame_id = "base_link"
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF Error: {e}")
                return None
        
        return grasp_pose
        
    def execute_pick_place(self, grasp_pose):
        if grasp_pose is None:
            rospy.logerr("Invalid grasp pose. Aborting pick and place.")
            return
        
        # Move to home position
        self.move_to_home()
        
        # Open gripper
        self.open_gripper()
        
        # Approach from above
        approach_pose = grasp_pose
        approach_pose.pose.position.z += 0.1  # 10cm above grasp position
        
        # Plan and execute approach
        self.arm_group.set_pose_target(approach_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if not success:
            rospy.logerr("Failed to reach approach position. Aborting pick and place.")
            return
        
        # Plan cartesian path to grasp position
        waypoints = []
        waypoints.append(grasp_pose.pose)  # Move down to grasp
        
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,       # eef_step
            0.0)        # jump_threshold
        
        if fraction < 0.9:
            rospy.logwarn(f"Only achieved {fraction:.2f} of Cartesian path. Aborting.")
            self.move_to_home()
            return
        
        # Execute cartesian path
        self.arm_group.execute(plan, wait=True)
        
        # Close gripper to grasp object
        self.close_gripper()
        rospy.sleep(0.5)  # Wait for gripper to close
        
        # Lift object
        lift_pose = grasp_pose
        lift_pose.pose.position.z += 0.1  # 10cm above grasp position
        
        self.arm_group.set_pose_target(lift_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if not success:
            rospy.logerr("Failed to lift object. Aborting pick and place.")
            return
        
        # Create place pose
        place_pose = PoseStamped()
        place_pose.header.frame_id = "base_link"
        place_pose.pose.position.x = self.place_position[0]
        place_pose.pose.position.y = self.place_position[1]
        place_pose.pose.position.z = self.place_position[2] + 0.1  # Add some height
        place_pose.pose.orientation = lift_pose.pose.orientation  # Keep same orientation
        
        # Move to place position
        self.arm_group.set_pose_target(place_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if not success:
            rospy.logerr("Failed to reach place position. Returning to home.")
            self.move_to_home()
            return
        
        # Lower object to surface
        place_down_pose = place_pose
        place_down_pose.pose.position.z = self.place_position[2]
        
        waypoints = []
        waypoints.append(place_down_pose.pose)
        
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,       # eef_step
            0.0)        # jump_threshold
        
        self.arm_group.execute(plan, wait=True)
        
        # Open gripper to release object
        self.open_gripper()
        rospy.sleep(0.5)  # Wait for gripper to open
        
        # Move away
        retreat_pose = place_down_pose
        retreat_pose.pose.position.z += 0.1  # Move up 10cm
        
        self.arm_group.set_pose_target(retreat_pose)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        # Return to home position
        self.move_to_home()
        
        rospy.loginfo("Pick and place operation completed successfully!")

if __name__ == '__main__':
    try:
        pick_place = PickAndPlace()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

## 5. Creating a Launch File for Pick and Place

Create a launch file `pick_place.launch` to run the complete system:

```xml
<launch>
  <!-- Start the perception system -->
  <include file="$(find kinetirover_perception)/launch/perception_pipeline.launch"/>
  
  <!-- Start MoveIt -->
  <include file="$(find px100_moveit)/launch/px100_moveit.launch">
    <arg name="use_actual" value="true"/>
  </include>
  
  <!-- Start the pick and place node -->
  <node name="pick_place_node" pkg="kinetirover_manipulation" type="darknet3d_pick_place.py" output="screen">
    <param name="target_object" value="bottle"/>
    <param name="place_position" value="[0.2, 0.2, 0.1]"/>
  </node>
  
  <!-- Start RViz with custom config -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find kinetirover_manipulation)/config/pick_place.rviz"/>
</launch>
```

## 6. Creating an RViz Configuration

To help visualize the perception pipeline, create an RViz configuration with the following displays:

1. RobotModel - to show the PX100 arm
2. TF - to visualize coordinate frames
3. Camera (RGB) - to show the camera view with YOLO detections
4. PointCloud2 - to visualize the point cloud data
5. MarkerArray - to visualize detected objects and 3D bounding boxes
6. PlanningScene - to show collision objects and planned trajectories

## 7. Training Custom YOLO Models

To improve detection for specific objects, you may want to train a custom YOLO model:

### 7.1 Data Collection

1. Collect 100-1000 images per object class
2. Include various angles, lighting conditions, and backgrounds
3. Use data augmentation to increase dataset variety

### 7.2 Annotation

1. Use LabelImg or a similar tool to annotate bounding boxes
2. Export annotations in YOLO format (classes, coordinates)

### 7.3 Training

```bash
# Clone Darknet repository
git clone https://github.com/AlexeyAB/darknet.git
cd darknet

# Modify Makefile to enable GPU, CUDNN, OPENCV
# Set GPU=1, CUDNN=1, OPENCV=1

# Compile
make

# Download pre-trained weights
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4.conv.137

# Configure training files
# - Create obj.names with class names
# - Create obj.data with paths and class count
# - Create yolov4-custom.cfg (modify from yolov4-custom.cfg)
# - Create train.txt and valid.txt with image paths

# Start training
./darknet detector train data/obj.data cfg/yolov4-custom.cfg yolov4.conv.137 -map
```

### 7.4 Converting Model for darknet_ros

1. Copy your trained weights file to `darknet_ros/yolo_network_config/weights/`
2. Copy your custom config file to `darknet_ros/yolo_network_config/cfg/`
3. Update your `obj.names` file to `darknet_ros/yolo_network_config/coco.names`
4. Modify `darknet_ros/config/ros.yaml` to use your custom files

## 8. Testing the Perception Pipeline

### 8.1 Setup Test Scene

1. Position the RealSense camera to view the workspace
2. Place objects from your trained classes on the table
3. Ensure proper lighting for optimal detection

### 8.2 Run the System

```bash
# In one terminal, launch the perception pipeline
roslaunch kinetirover_perception perception_pipeline.launch

# In another terminal, monitor detections
rostopic echo /darknet_ros_3d/bounding_boxes

# Once confirmed working, run the pick and place demo
roslaunch kinetirover_manipulation pick_place.launch
```

### 8.3 Monitoring and Debugging

1. Check if 2D detections are working:
   ```bash
   rostopic echo /darknet_ros/bounding_boxes
   ```

2. Check if 3D bounding boxes are generated:
   ```bash
   rostopic echo /darknet_ros_3d/bounding_boxes
   ```

3. Watch RViz for visual confirmation of detections and planned paths

4. Check TF transformations if object positions seem incorrect:
   ```bash
   rosrun tf tf_echo base_link camera_color_optical_frame
   ```

## 9. Troubleshooting

### 9.1 Common Issues

1. **Object detection not working**
   - Check camera is properly connected
   - Verify lighting conditions
   - Ensure YOLO model is properly loaded

2. **3D localization errors**
   - Check camera calibration
   - Verify TF transformations between camera and robot
   - Adjust minimum_probability and minimum_detection_threshold parameters

3. **Gripper fails to grasp objects**
   - Check object size is within gripper capacity
   - Adjust grasp pose offsets
   - Verify approach direction

### 9.2 Advanced Debugging

1. Use `rosrun rqt_image_view rqt_image_view` to visualize camera feeds
2. Use `rqt_tf_tree` to verify TF frame relationships
3. Use `rostopic hz /darknet_ros_3d/bounding_boxes` to check detection rate

## 10. Performance Optimization

1. **Reduce point cloud resolution** to improve processing speed
2. **Increase minimum_probability threshold** to filter out low-confidence detections
3. **Limit detection area** using a region of interest
4. **Use a lighter YOLO model** (like YOLOv4-tiny) for faster inference

## Conclusion

In this tutorial, we've implemented a complete perception pipeline using Darknet YOLO for 2D object detection and 3D object localization using the gb_visual_detection_3d package. We've integrated this with MoveIt to enable pick and place operations with the PX100 robotic arm. The system can be extended with custom object detection models and optimized for specific applications.

---
*Previous: [RealSense Setup](realsense-setup.md) | Next: [MoveIt Integration](moveit-setup.md)*