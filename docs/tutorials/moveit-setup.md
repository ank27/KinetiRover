# MoveIt Integration for PX100 Manipulator

*Previous: [Perception Pipeline](perception-pipeline.md) | [Main Page](../index.md)*

## Overview

This tutorial covers how to use MoveIt with the PX100 manipulator for motion planning and control. MoveIt is a powerful framework for manipulation, 3D perception, kinematics, control, and navigation. We'll explain how to use our pre-configured MoveIt setup with both the physical robot and in simulation with Gazebo.

## Prerequisites

Before starting this tutorial, ensure you have:

- Completed the [Getting Started](getting-started.md) tutorial
- Understood the [PX100 Robot Description](robot-description.md)
- Installed MoveIt with: `sudo apt install ros-noetic-moveit`

## Understanding MoveIt Architecture

MoveIt consists of several components working together:

1. **Move Group**: The primary ROS node that integrates motion planning, kinematics, collision checking, and control
2. **RViz Plugin**: For interactive motion planning and visualization
3. **Planning Scene**: Represents the environment and robot state
4. **Planning Pipeline**: Configuration for motion planning algorithms (OMPL, CHOMP, etc.)
5. **Controllers**: Interface with hardware or simulation for executing plans

## PX100 MoveIt Configuration

Our `px100_moveit` package contains the configuration for using MoveIt with the PX100 manipulator. It was generated using the MoveIt Setup Assistant and includes:

```
px100_moveit/
├── config/
│   ├── fake_controllers/
│   ├── 4dof_controllers.yaml
│   ├── 4dof_joint_limits.yaml
│   ├── chomp_planning.yaml
│   ├── kinematics.yaml
│   └── ompl_planning.yaml
├── launch/
│   ├── chomp_planning_pipeline.launch.xml
│   ├── move_group.launch
│   ├── moveit_rviz.launch
│   ├── ompl_planning_pipeline.launch.xml
│   ├── planning_context.launch
│   ├── px100_moveit.launch
│   └── trajectory_execution.launch.xml
└── config/
    └── px100.srdf.xacro
```

## Launching MoveIt

There are two main ways to use MoveIt with the PX100:

1. With a simulated robot in Gazebo
2. With the actual physical robot

### Using MoveIt with Gazebo Simulation

To launch MoveIt with Gazebo simulation:

```bash
roslaunch px100_moveit px100_moveit.launch use_gazebo:=true
```

This command will:
1. Start Gazebo with the PX100 model
2. Launch the MoveIt Move Group node
3. Start RViz with the MoveIt plugin

The key parameters in this setup:

- `use_gazebo:=true`: Tells the system to use Gazebo for simulation
- `use_fake:=false`: Disables the fake controllers (not needed with Gazebo)
- `use_actual:=false`: Disables connections to actual hardware

### Using MoveIt with the Physical Robot

To use MoveIt with the actual PX100 hardware:

```bash
roslaunch px100_moveit px100_moveit.launch use_actual:=true
```

This command will:
1. Initialize connections to the Dynamixel motors
2. Start the MoveIt Move Group node
3. Launch RViz with the MoveIt plugin

Important parameters for hardware usage:

- `use_actual:=true`: Enables communication with the physical robot
- `use_gazebo:=false`: Disables Gazebo simulation
- `use_fake:=false`: Disables fake controllers

## Understanding px100_moveit_impl Package

The `px100_moveit_impl` package provides higher-level interfaces for controlling the PX100 using MoveIt. This package contains C++ and Python implementations for:

1. Controlling the robot in joint space
2. Creating and executing trajectories
3. Performing pick and place tasks

The key components are:

### move_group_interface.h/cpp

This C++ implementation provides a class `RobotJointMoveInterface` that wraps the MoveIt C++ API with simplified methods for:

```cpp
// Joint position control
bool move_arm_to_joint_position(std::vector<double> joints_position);

// Predefined positions
bool move_arm_to_home();
bool move_arm_to_sleep();
bool move_arm_to_upright();

// Gripper control
bool open_gripper();
bool close_gripper();

// Cartesian space movement
void set_ee_trajectory(std::vector<geometry_msgs::Pose> waypoints);
bool move_ee_pose(geometry_msgs::Pose& pose);
```

These methods allow you to:

1. Move the arm to specific joint positions
2. Move to predefined configurations (home, sleep, upright)
3. Control the gripper
4. Move the end-effector in Cartesian space

## How MoveIt Generates Trajectories

MoveIt uses a pipeline to generate trajectories that consists of several steps:

1. **Planning Request**: The user specifies a goal position (joint space or Cartesian space)
2. **Motion Planning**: A planning algorithm (e.g., RRT, PRM) finds a collision-free path
3. **Trajectory Processing**: The raw path is converted to a time-parameterized trajectory
4. **Execution**: The trajectory is sent to controllers

### Planning in Joint Space

When planning in joint space, you specify the target joint angles directly:

```cpp
// Example from move_group_interface.cpp
std::vector<double> joint_pose = {0.0, -1.5, 1.5, 0.8}; // For positions: waist, shoulder, elbow, wrist_angle
robotJointMoveInterface.move_arm_to_joint_position(joint_pose);
```

Advantages of joint space planning:
- More predictable motion
- No inverse kinematics calculation needed
- Guaranteed to be achievable by the robot

### Planning in Cartesian Space

For Cartesian space planning, you specify the end-effector pose (position and orientation):

```cpp
// Example from move_group_interface.cpp
geometry_msgs::Pose target_pose;
target_pose.position.x = 0.2;
target_pose.position.y = 0.0;
target_pose.position.z = 0.2;

// Set orientation with quaternion
tf2::Quaternion q;
q.setRPY(0, 0, 0); // Roll, Pitch, Yaw
q.normalize();
target_pose.orientation.x = q[0];
target_pose.orientation.y = q[1];
target_pose.orientation.z = q[2];
target_pose.orientation.w = q[3];

robotJointMoveInterface.move_ee_pose(target_pose);
```

## Understanding Quaternions

Quaternions are a mathematical concept used to represent 3D rotations. While Euler angles (roll, pitch, yaw) are more intuitive, quaternions avoid issues like gimbal lock and provide smoother interpolation.

A quaternion consists of four components: `q = [x, y, z, w]` where:
- `x, y, z` represent the axis of rotation (multiplied by sine of half the rotation angle)
- `w` represents the cosine of half the rotation angle

### Converting Between Representations

In our code, we often convert between Euler angles and quaternions:

```cpp
// Euler angles to quaternion
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw); // In radians
q.normalize();

// Access components
target_pose.orientation.x = q[0];
target_pose.orientation.y = q[1];
target_pose.orientation.z = q[2];
target_pose.orientation.w = q[3];
```

## Creating Waypoints

For complex trajectories, you can define a series of waypoints in Cartesian space:

```cpp
std::vector<geometry_msgs::Pose> waypoints;

// Start with current pose
geometry_msgs::Pose current_pose = robotJointMoveInterface.getCurrentEndeffectorPose();
waypoints.push_back(current_pose);

// Define additional waypoints
geometry_msgs::Pose target_pose1 = current_pose;
target_pose1.position.z += 0.1; // Move 10cm up
waypoints.push_back(target_pose1);

geometry_msgs::Pose target_pose2 = target_pose1;
target_pose2.position.x += 0.1; // Move 10cm forward
waypoints.push_back(target_pose2);

// Execute trajectory through waypoints
robotJointMoveInterface.set_ee_trajectory(waypoints);
```

MoveIt will generate a trajectory that passes through all waypoints while respecting joint limits and avoiding collisions.

## Planning Algorithms and Kinematics Configuration

The PX100 MoveIt configuration includes several planning and kinematics options that can be customized.

### OMPL Planning

The Open Motion Planning Library (OMPL) provides a variety of sampling-based planning algorithms:

- **RRT (Rapidly-exploring Random Tree)**: Fast but jerky paths
- **RRTConnect**: Bi-directional RRT, usually more efficient
- **PRM (Probabilistic Roadmap)**: Good for repeated planning in the same space
- **TRRT (Transition-based RRT)**: Better for planning with cost functions

Our default configuration uses RRTConnect for its good balance of speed and quality.

### Kinematics Solvers

Kinematics solvers calculate the relationship between joint positions and end-effector poses:

#### KDL Kinematics Solver

The Kinematics and Dynamics Library (KDL) solver is the standard solver in MoveIt:

```yaml
px100_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  position_only_ik: false
```

#### TRAC-IK Solver

A drop-in replacement for KDL that is faster and more reliable:

```yaml
px100_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.005
  solve_type: Speed
  position_only_ik: true
```

#### LMA Solver

A solver based on the Levenberg-Marquardt algorithm:

```yaml
px100_arm:
  kinematics_solver: lma_kinematics_plugin/LMAKinematicsPlugin
  kinematics_solver_search_resolution: 0.010
  kinematics_solver_timeout: 0.010
  position_only_ik: true
```

### Position-only IK vs Complete IK

- **position_only_ik: true** - Finds joint solutions that match only the end-effector position, ignoring orientation
- **position_only_ik: false** - Finds joint solutions that match both position and orientation

For the PX100, which has only 4 DOF, position-only IK is often necessary since the robot cannot achieve arbitrary orientations due to its limited DOF.

## Practical Examples

### Example 1: Moving to a Predefined Position

```cpp
// Move to the "home" position
robotJointMoveInterface.move_arm_to_home();

// Move to the "sleep" position
robotJointMoveInterface.move_arm_to_sleep();
```

### Example 2: Simple Pick and Place

```cpp
// Step 1: Move to pre-grasp position
geometry_msgs::Pose pre_grasp_pose;
pre_grasp_pose.position.x = 0.2;
pre_grasp_pose.position.y = 0.0;
pre_grasp_pose.position.z = 0.15;
tf2::Quaternion q;
q.setRPY(0, M_PI/2, 0);
q.normalize();
pre_grasp_pose.orientation.x = q[0];
pre_grasp_pose.orientation.y = q[1];
pre_grasp_pose.orientation.z = q[2];
pre_grasp_pose.orientation.w = q[3];
robotJointMoveInterface.move_ee_pose(pre_grasp_pose);

// Step 2: Open gripper
robotJointMoveInterface.open_gripper();

// Step 3: Move down to grasp
geometry_msgs::Pose grasp_pose = pre_grasp_pose;
grasp_pose.position.z -= 0.05;
robotJointMoveInterface.move_ee_pose(grasp_pose);

// Step 4: Close gripper
robotJointMoveInterface.close_gripper();

// Step 5: Lift object
robotJointMoveInterface.move_ee_pose(pre_grasp_pose);

// Step 6: Move to place position
geometry_msgs::Pose place_pose = pre_grasp_pose;
place_pose.position.y = 0.2;
robotJointMoveInterface.move_ee_pose(place_pose);

// Step 7: Open gripper to release
robotJointMoveInterface.open_gripper();
```

### Example 3: Using Python Interface (moveit_python_example.py)

The PX100 MoveIt setup also includes a Python interface:

```python
# Initialize the MoveGroupCommander
import moveit_commander
robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("px100_arm")
gripper_group = moveit_commander.MoveGroupCommander("px100_gripper")

# Plan and execute a joint space goal
joint_goal = arm_group.get_current_joint_values()
joint_goal[0] = 0.2  # waist
joint_goal[1] = -0.5  # shoulder
joint_goal[2] = 0.5  # elbow
joint_goal[3] = 0.0  # wrist_angle
arm_group.go(joint_goal, wait=True)

# Plan and execute a Cartesian goal
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.25
pose_goal.position.y = 0.0
pose_goal.position.z = 0.2
arm_group.set_pose_target(pose_goal)
arm_group.go(wait=True)
```

## Troubleshooting

### Common Issues and Solutions

1. **No IK solution found**
   - Try using position_only_ik: true in kinematics.yaml
   - Ensure the target pose is within the robot's workspace
   - Try a different kinematics solver (KDL, TRAC-IK, LMA)

2. **Controller failed during execution**
   - Check if the robot is in a singular configuration
   - Verify that all controllers are running: `rosservice call /controller_manager/list_controllers`
   - Check for hardware issues like servo overload

3. **Motion Planning Failed**
   - Increase planning time: `arm_group.set_planning_time(10.0)`
   - Try a different planning algorithm
   - Check if the goal is collision-free

4. **Path Constraints Not Satisfied**
   - Simplify or remove path constraints
   - Increase the constraint tolerance

## Conclusion

MoveIt provides a powerful framework for planning and executing motions with the PX100 manipulator. By understanding the components, configuration options, and interfaces available, you can develop sophisticated robotic applications for manipulation tasks.

Experiment with different planning approaches (joint space vs. Cartesian space), kinematics solvers, and planning algorithms to find the best configuration for your specific application.

---

*Previous: [Perception Pipeline](perception-pipeline.md) | [Main Page](../index.md)*