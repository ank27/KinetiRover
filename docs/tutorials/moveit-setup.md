# MoveIt Integration: Motion Planning and Pick & Place

*Previous: [Perception Pipeline](perception-pipeline.md) | Next: Advanced Topics*

## Overview

This tutorial covers how to integrate the PX100 robot with MoveIt for motion planning and pick & place operations. We'll use the pose data from the perception pipeline to grasp objects detected by the camera.

## Prerequisites

- Complete the [Gazebo Simulation](gazebo-simulation.md) tutorial
- Complete the [Perception Pipeline](perception-pipeline.md) tutorial
- ROS Noetic installed
- MoveIt packages installed

## Installation

### 1. Install MoveIt

```bash
sudo apt-get install ros-noetic-moveit
```

## Setting Up MoveIt for PX100

### 1. Create a new package for MoveIt integration

```bash
cd ~/kinetibot_ws/src
catkin_create_pkg px100_moveit_interface roscpp rospy moveit_ros_planning_interface actionlib moveit_msgs geometry_msgs
cd px100_moveit_interface
mkdir -p scripts launch
```

### 2. Create the pick and place node

Create a new Python script for implementing pick and place functionality:

```bash
touch scripts/px100_pick_place.py
chmod +x scripts/px100_pick_place.py
```

Add the following code to implement a pick and place node that subscribes to the target pose from the perception pipeline:

```python
#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
import math
import numpy as np

class PX100PickPlace:
    def __init__(self):
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('px100_pick_place', anonymous=True)
        
        # Setup robot and scene
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("px100_arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("px100_gripper")
        
        # Set parameters
        self.arm_group.set_planning_time(5.0)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        
        # Define arm positions
        self.home_position = [0.0, 0.0, 0.0, 0.0]
        self.sleep_position = [0.0, -1.88, 1.5, 0.8]
        self.approach_offset = 0.1  # 10cm approach offset
        
        # Target object variables
        self.target_pose = None
        self.has_target = False
        
        # Gripper parameters
        self.gripper_open_pos = [0.037, -0.037]  # Open position [left_finger, right_finger]
        self.gripper_closed_pos = [0.015, -0.015]  # Closed position
        
        # Subscribe to target pose
        rospy.Subscriber('/px100/target_pose', geometry_msgs.msg.PoseStamped, self.target_callback)
        
        # Setup complete
        rospy.loginfo("PX100 Pick and Place node initialized")
        
    def target_callback(self, msg):
        self.target_pose = msg
        self.has_target = True
        rospy.loginfo(f"Received new target: position [{msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}]")
        
    def open_gripper(self):
        self.gripper_group.go(self.gripper_open_pos, wait=True)
        self.gripper_group.stop()
        rospy.loginfo("Gripper opened")
    
    def close_gripper(self):
        self.gripper_group.go(self.gripper_closed_pos, wait=True)
        self.gripper_group.stop()
        rospy.loginfo("Gripper closed")
    
    def move_arm_to_pose(self, pose, allow_replanning=True):
        self.arm_group.set_pose_target(pose)
        self.arm_group.allow_replanning(allow_replanning)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success
    
    def move_arm_to_joint_positions(self, joint_positions):
        self.arm_group.go(joint_positions, wait=True)
        self.arm_group.stop()
        rospy.loginfo(f"Moved to joint positions: {joint_positions}")
    
    def move_to_home(self):
        self.move_arm_to_joint_positions(self.home_position)
    
    def move_to_sleep(self):
        self.move_arm_to_joint_positions(self.sleep_position)
    
    def create_approach_pose(self, target_pose):
        approach_pose = geometry_msgs.msg.PoseStamped()
        approach_pose.header = target_pose.header
        approach_pose.pose = target_pose.pose
        
        # Add offset in Z direction for approach
        approach_pose.pose.position.z += self.approach_offset
        
        return approach_pose
    
    def perform_pick_and_place(self):
        # Check if we have a target
        if not self.has_target:
            rospy.logwarn("No target pose received yet")
            return False
        
        target_pose = self.target_pose  # Local copy to avoid race conditions
        
        # Create approach pose
        approach_pose = self.create_approach_pose(target_pose)
        
        # Step 1: Open gripper
        self.open_gripper()
        
        # Step 2: Move to approach position
        rospy.loginfo("Moving to approach position")
        if not self.move_arm_to_pose(approach_pose):
            rospy.logerr("Failed to move to approach position")
            return False
        
        # Step 3: Move to target position
        rospy.loginfo("Moving to target position")
        if not self.move_arm_to_pose(target_pose):
            rospy.logerr("Failed to move to target position")
            return False
        
        # Step 4: Close gripper
        self.close_gripper()
        rospy.sleep(0.5)  # Short delay to ensure grip
        
        # Step 5: Move back to approach position
        rospy.loginfo("Moving back to approach position")
        if not self.move_arm_to_pose(approach_pose):
            rospy.logerr("Failed to move back to approach position")
            # Continue anyway, don't return
        
        # Step 6: Move to place position
        place_pose = geometry_msgs.msg.PoseStamped()
        place_pose.header = target_pose.header
        place_pose.pose.position.x = 0.15
        place_pose.pose.position.y = 0.15
        place_pose.pose.position.z = 0.05
        place_pose.pose.orientation = target_pose.pose.orientation
        
        rospy.loginfo("Moving to place position")
        if not self.move_arm_to_pose(place_pose):
            rospy.logerr("Failed to move to place position")
            # Continue anyway, don't return
        
        # Step 7: Open gripper to release object
        self.open_gripper()
        
        # Step 8: Move back to home position
        self.move_to_home()
        
        return True

if __name__ == '__main__':
    try:
        pick_place = PX100PickPlace()
        
        # Wait a moment to initialize everything
        rospy.sleep(2.0)
        
        # Move to home position
        pick_place.move_to_home()
        
        rate = rospy.Rate(1.0)  # Check for targets at 1Hz
        while not rospy.is_shutdown():
            if pick_place.has_target:
                pick_place.perform_pick_and_place()
                pick_place.has_target = False  # Reset after execution
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
```

### 3. Create a launch file

Create a launch file to start the MoveIt interface and pick & place node:

```bash
touch launch/px100_pick_place.launch
```

Add the following content:

```xml
<launch>
  <!-- Launch MoveIt for PX100 -->
  <include file="$(find px100_moveit)/launch/px100_moveit.launch">
    <arg name="use_actual" value="true" />
  </include>
  
  <!-- Launch perception pipeline -->
  <include file="$(find px100_perception)/launch/px100_perception.launch" />
  
  <!-- Launch pick and place node -->
  <node name="px100_pick_place" pkg="px100_moveit_interface" type="px100_pick_place.py" output="screen" />
</launch>
```

### 4. Create a simulated version launch file

Create a launch file for testing in Gazebo:

```bash
touch launch/px100_pick_place_sim.launch
```

Add the following content:

```xml
<launch>
  <!-- Launch Gazebo simulation -->
  <include file="$(find px100_gazebo)/launch/px100_gazebo.launch" />
  
  <!-- Launch MoveIt for PX100 with Gazebo -->
  <include file="$(find px100_moveit)/launch/px100_moveit.launch">
    <arg name="use_gazebo" value="true" />
    <arg name="use_actual" value="false" />
  </include>
  
  <!-- Launch perception pipeline -->
  <include file="$(find px100_perception)/launch/px100_perception.launch" />
  
  <!-- Launch pick and place node -->
  <node name="px100_pick_place" pkg="px100_moveit_interface" type="px100_pick_place.py" output="screen" />
</launch>
```

## Running MoveIt with Pick and Place

### Simulation Mode

```bash
# Make sure your workspace is built and sourced
cd ~/kinetibot_ws
catkin_make
source devel/setup.bash

# Launch in simulation mode
roslaunch px100_moveit_interface px100_pick_place_sim.launch
```

### Real Hardware Mode

```bash
# Make sure your workspace is built and sourced
cd ~/kinetibot_ws
catkin_make
source devel/setup.bash

# Launch with real hardware
roslaunch px100_moveit_interface px100_pick_place.launch
```

## Testing Pick and Place

1. Place an object (like a cube or small box) in front of the RealSense camera
2. The perception pipeline will detect the object and publish its pose
3. The pick and place node will:
   - Move to the approach position
   - Move to the object position
   - Close the gripper
   - Lift the object
   - Move to the place position
   - Release the object
   - Return to home position

## Troubleshooting

### Planning Issues

If the robot fails to plan a path to the target:

1. Make sure the target pose is reachable by the robot
2. Increase planning time and attempts in the code:
```python
self.arm_group.set_planning_time(10.0)  # Increase to 10 seconds
self.arm_group.set_num_planning_attempts(20)  # Increase to 20 attempts
```

### Perception Issues

If the robot doesn't detect objects correctly:

1. Make sure the lighting is adequate
2. Check the camera alignment
3. Verify that the object is within the detection range
4. Adjust the minimum_probability parameter in the perception launch file

## Next Steps

Now that you have a working pick and place system, you can:

1. Improve perception with custom object detection models
2. Add collision objects to the planning scene
3. Implement more complex manipulation tasks
4. Add a sequence of operations for sorting objects

---

*Previous: [Perception Pipeline](perception-pipeline.md) | Next: Advanced Topics*
