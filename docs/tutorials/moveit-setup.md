# MoveIt Integration

*Previous: [Gazebo Simulation](gazebo-simulation.md) | Next: [RealSense Integration](realsense-setup.md)*

## Overview

This tutorial covers setting up and using MoveIt with the PX100 robot manipulator. MoveIt is an advanced motion planning framework for ROS that simplifies complex robot manipulation tasks. We'll walk through the setup process, configuration, and basic usage patterns.

## Prerequisites

Before continuing, ensure you have:
- Completed the [Gazebo Simulation](gazebo-simulation.md) tutorial
- MoveIt installed on your system
```bash
sudo apt install ros-noetic-moveit -y
```

## MoveIt Setup Assistant

MoveIt provides a graphical tool called the Setup Assistant to help create the necessary configuration files for your robot. While the PX100 already has these configs in the repository, this section explains how they were created and how you could modify them.

### Running the Setup Assistant

1. Launch the Setup Assistant:
```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```

2. In the Start screen, click on "Create New MoveIt Configuration Package" or "Edit Existing MoveIt Configuration Package"

3. If creating a new configuration:
   - Load the robot model by clicking "Browse" and selecting the PX100 URDF file
   - Click "Load Files"

### Setup Steps

The Setup Assistant guides you through several steps to configure MoveIt for your robot:

1. **Self-Collision Checking**: Generates a collision matrix to disable collision checking between parts that can't collide
   
2. **Virtual Joints**: Defines how the robot connects to the world

3. **Planning Groups**: Defines groups of joints for motion planning
   - For PX100, we define two groups:
     - `px100_arm`: Contains all arm joints
     - `px100_gripper`: Contains gripper joints

4. **Robot Poses**: Defines named poses for the robot (like "home", "ready", etc.)

5. **End Effectors**: Defines the robot's end effectors
   - For PX100, we define the gripper as an end effector

6. **Passive Joints**: Specify joints that are not actively controlled

7. **Author Information**: Add metadata about the configuration author

8. **Configuration Files**: Generate the configuration files

### Generated Configuration Files

The Setup Assistant generates the following directory structure:

```
px100_moveit/
├── config/
│   ├── controllers.yaml
│   ├── fake_controllers.yaml
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── ompl_planning.yaml
│   └── sensors_3d.yaml
├── launch/
│   ├── demo.launch
│   ├── move_group.launch
│   ├── planning_context.launch
│   └── ...
└── ...
```

## Using the PX100 MoveIt Configuration

The KinetiRover repository already includes a pre-configured MoveIt package. You can use it directly:

```bash
roslaunch px100_moveit px100_moveit.launch use_rviz:=true
```

This command launches:
1. The robot model
2. The MoveIt planning and execution components
3. RViz with the MoveIt plugin for visualization and interaction

## Planning and Executing Motions

### Using RViz Interface

1. **Interactive Markers**: Use the interactive markers (arrow, ring) to position the end effector
2. **Motion Planning Panel**: Configure planning parameters
3. **Plan & Execute**: Click "Plan" to generate a trajectory, then "Execute" to run it

### Planning to Named Poses

1. In the RViz Motion Planning panel, select a named pose from the dropdown
2. Click "Plan and Execute" to move the robot to that pose

### Programming with MoveIt

MoveIt provides a C++ and Python API for programmatic control. Below is a simple Python example to move the arm to a target position:

```python
#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# Initialize moveit_commander and a rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

# Instantiate a RobotCommander object
robot = moveit_commander.RobotCommander(robot_description="/px100/robot_description")

# Instantiate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object
arm_group = moveit_commander.MoveGroupCommander("px100_arm", robot_description="/px100/robot_description")
gripper_group = moveit_commander.MoveGroupCommander("px100_gripper", robot_description="/px100/robot_description")

# We can get the name of the reference frame for this robot
planning_frame = arm_group.get_planning_frame()
print("Planning frame: %s" % planning_frame)

# We can get the name of the end-effector link for this group
eef_link = arm_group.get_end_effector_link()
print("End effector link: %s" % eef_link)

# Move to home position
arm_group.set_named_target("Home")
arm_group.go(wait=True)

# Open gripper
gripper_group.set_named_target("Open")
gripper_group.go(wait=True)

# Plan and execute a path to a pose goal
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.2
pose_goal.position.y = 0.0
pose_goal.position.z = 0.1

arm_group.set_pose_target(pose_goal)
arm_group.go(wait=True)

# Clean up
arm_group.clear_pose_targets()

# Close gripper
gripper_group.set_named_target("Closed")
gripper_group.go(wait=True)

# Return to home position
arm_group.set_named_target("Home")
arm_group.go(wait=True)

# Shut down moveit_commander
moveit_commander.roscpp_shutdown()
```

## Using Joint Trajectory Controllers

The PX100 uses joint trajectory controllers to execute planned motions. These controllers are configured in the `config/controllers.yaml` file and provide a ROS action interface for executing planned trajectories.

### Controller Configuration

The PX100 MoveIt configuration uses two controllers:

1. **arm_controller**: Controls the arm joints (waist, shoulder, elbow, wrist_angle)
2. **gripper_controller**: Controls the gripper (left_finger, right_finger)

## Advanced MoveIt Features

### Path Constraints

You can add constraints to the motion planning, such as orientation constraints, position constraints, etc.

```python
# Add an orientation constraint
constr = moveit_msgs.msg.Constraints()
orientation_constraint = moveit_msgs.msg.OrientationConstraint()
orientation_constraint.header.frame_id = "base_link"
orientation_constraint.link_name = arm_group.get_end_effector_link()
orientation_constraint.orientation = pose_goal.orientation
orientation_constraint.absolute_x_axis_tolerance = 0.1
orientation_constraint.absolute_y_axis_tolerance = 0.1
orientation_constraint.absolute_z_axis_tolerance = 0.1
orientation_constraint.weight = 1.0
constr.orientation_constraints.append(orientation_constraint)
arm_group.set_path_constraints(constr)
```

### Custom Planning Pipelines

MoveIt supports different planning pipelines like OMPL, CHOMP, and STOMP. You can configure and select these in the `ompl_planning.yaml` file or through the API.

```python
# Select a specific planner
arm_group.set_planner_id("RRTConnectkConfigDefault")
```

## Integration with Perception

In later tutorials, we'll explore integrating MoveIt with perception using the RealSense camera for tasks like object detection and manipulation.

## MoveIt Resources

For more detailed information about MoveIt, refer to the official documentation and tutorials:

- [MoveIt Official Tutorials](https://ros-planning.github.io/moveit_tutorials/)
- [MoveIt API Documentation](https://moveit.ros.org/documentation/)

## Troubleshooting

### Common Issues

1. **Planning Failures**
   - Ensure the target pose is within the robot's workspace
   - Check for collision objects in the scene
   - Try different planners

2. **Execution Failures**
   - Verify controller configuration
   - Check joint limits in the URDF match hardware capabilities

3. **Slow Planning**
   - Adjust planning time limits
   - Consider using a different planner

## Next Steps

Now that you're familiar with MoveIt and the PX100's motion planning capabilities, you're ready to move on to integrating perception with the RealSense camera in the next tutorial.

---

*Previous: [Gazebo Simulation](gazebo-simulation.md) | Next: [RealSense Integration](realsense-setup.md)*