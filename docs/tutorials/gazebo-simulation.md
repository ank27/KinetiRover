# Gazebo Simulation for PX100

*Previous: [Robot Description](robot-description.md) | Next: [RealSense Integration](realsense-setup.md)*

## Overview

This tutorial explains how to set up and run the PX100 robot in the Gazebo simulation environment. Gazebo provides a robust physics engine that allows us to simulate robot behaviors in a realistic way before deploying to physical hardware. We'll explore the simulation package structure, controller types, and how to effectively use Gazebo with the PX100 manipulator.

## PX100 Gazebo Package Structure

The `px100_gazebo` package contains all the necessary files to simulate the PX100 robot in Gazebo:

```
px100_gazebo/
├── config/
│   ├── position_controllers/
│   │   └── px100_position_controllers.yaml
│   └── trajectory_controllers/
│       └── px100_trajectory_controllers.yaml
├── launch/
│   └── px100_gazebo.launch
├── plugin/
│   └── _d435.gazebo.xacro
├── worlds/
│   └── px100_gazebo.world
├── CMakeLists.txt
└── package.xml
```

### Key Components

1. **config**: Contains controller configuration files
2. **launch**: Contains launch files for starting the simulation
3. **plugin**: Contains plugins for the simulation (like the RealSense D435 camera plugin)
4. **worlds**: Contains Gazebo world files that define the simulation environment

## Understanding Controllers in Gazebo

Before diving into the simulation setup, it's important to understand the types of controllers used in ROS with Gazebo.

### Position Controllers vs. Trajectory Controllers

#### Position Controllers

**Purpose**: Control individual joints by directly commanding positions.

**Configuration**: Defined in `config/position_controllers/px100_position_controllers.yaml`

```yaml
# Publish all joint states
# Creates the /joint_states topic necessary in ROS
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

waist_controller:
  type: effort_controllers/JointPositionController
  joint: waist
  pid: {p: 50, i: 0.0, d: 0.0}

shoulder_controller:
  type: effort_controllers/JointPositionController
  joint: shoulder
  pid: {p: 100, i: 0.0, d: 0.0}

# Additional joint controllers follow...
```

**Use Cases**:
- Direct control of individual joints
- Testing and debugging
- Simple applications where coordinated motion is not required
- Manual control via command line or scripts

**Example Usage**:
```bash
# To command the waist joint to position 1.0 radians
rostopic pub /px100/waist_controller/command std_msgs/Float64 "data: 1.0"
```

#### Trajectory Controllers

**Purpose**: Control coordinated motion across multiple joints through timestamped waypoints.

**Configuration**: Defined in `config/trajectory_controllers/px100_trajectory_controllers.yaml`

```yaml
# Publish all joint states
# Creates the /joint_states topic necessary in ROS
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 100

arm_controller:
  type: effort_controllers/JointTrajectoryController
  joints:
    - waist
    - shoulder
    - elbow
    - wrist_angle
  gains: # Required because we're controlling an effort interface
    waist: {p: 100.0,  d: 0.5, i: 5.0}
    shoulder: {p: 500.0,  d: 0.5, i: 10.0}
    elbow: {p: 300.0,  d: 0.0, i: 10.0}
    wrist_angle: {p: 100.0,  d: 0.0, i: 3.0}

  constraints:
    goal_time: 0.2
    waist:
      goal: 0.1
      trajectory: 0.2
    # Additional joint constraints follow...
```

**Use Cases**:
- Complex, coordinated motions
- Integration with MoveIt (MoveIt outputs trajectories)
- Applications requiring smooth, time-parameterized movements
- Path planning and execution

**Why Use with MoveIt**: 

MoveIt generates motion plans as trajectories - sequences of joint positions over time. The trajectory controller accepts these plans directly and executes them while maintaining timing constraints. This is essential for coordinated movements and collision-free motion planning.

## Controller Manager

The Controller Manager acts as an interface between ROS controllers and the Gazebo simulator. It plays several crucial roles in the simulation setup:

1. **Loading Controllers**: Loads controller definitions from YAML files into the ROS parameter server

2. **Starting/Stopping Controllers**: Manages the lifecycle of controllers, allowing them to be started, stopped, or switched during runtime

3. **Resource Conflict Management**: Ensures that multiple controllers don't try to command the same hardware resources simultaneously

4. **Interface Translation**: Translates between the abstract controller interfaces and the specific hardware interfaces provided by Gazebo

In the launch file, the controller manager is started as follows:

```xml
<node
  name="controller_spawner"
  pkg="controller_manager"
  type="controller_manager"
  respawn="false"
  output="screen"
  ns="$(arg robot_name)"
  args="spawn arm_controller gripper_controller joint_state_controller"/>
```

This node spawns (loads and starts) three controllers: arm_controller, gripper_controller, and joint_state_controller.

## Understanding Gazebo ROS Integration

### gazebo_ros Package

The `gazebo_ros` package provides the interface between ROS and Gazebo. It serves several important functions:

1. **Communication Bridge**: Enables ROS nodes to communicate with the Gazebo simulation environment

2. **World Management**: Allows loading, pausing, and resetting Gazebo worlds through ROS services

3. **Plugin Framework**: Provides plugins for adding sensors, controllers, and custom physics to Gazebo models

4. **Launch Integration**: Offers launch files and tools for starting Gazebo with ROS integration

In our simulation setup, we use it to:
- Launch the Gazebo simulator with ROS integration
- Load a specific world file
- Enable ROS services for interacting with the simulation

### URDF Spawner

The URDF Spawner is a Gazebo ROS tool that loads robot models from URDF descriptions into the Gazebo simulator. It's a critical component for simulation because:

1. **Dynamic Loading**: It allows robots to be loaded into an already-running Gazebo instance

2. **Namespace Management**: It can place the robot's ROS topics in specific namespaces

3. **Positioning**: It can specify the robot's initial position and orientation in the world

4. **Parameter Integration**: It retrieves the robot description from the ROS parameter server

In our launch file, the URDF Spawner is used as follows:

```xml
<node
  name="urdf_spawner"
  pkg="gazebo_ros"
  type="spawn_model"
  respawn="false"
  output="screen"
  ns="$(arg robot_name)"
  args="-urdf -model $(arg robot_model) -param robot_description"/>
```

This node loads the robot description from the `robot_description` parameter and spawns it in Gazebo as a model named according to `robot_model`.

## Unpausing the Physics Engine

When Gazebo starts, the physics engine is often paused by default to allow time for loading models and setting up the environment. This is controlled by the `paused` argument in the launch file:

```xml
<arg name="paused" default="true"/>
```

When set to `true`, the simulation starts in a paused state. This is useful because:

1. **Initialization Time**: It gives all nodes and controllers time to initialize properly

2. **Resource Management**: It prevents computation-intensive physics calculations during setup

3. **Deterministic Start**: It ensures that simulation starts from a consistent state

To unpause the physics engine and allow the simulation to run, you can either:

1. **Use the Gazebo GUI**: Click the play button in the Gazebo user interface

2. **Use a ROS Service**: Call the `/gazebo/unpause_physics` service
   ```bash
   rosservice call /gazebo/unpause_physics
   ```

3. **Set the Launch Parameter**: Change the launch parameter to `paused:=false`
   ```bash
   roslaunch px100_gazebo px100_gazebo.launch paused:=false
   ```

## Running the PX100 in Gazebo

Now let's put it all together and run the PX100 in Gazebo simulation.

### Launch the Simulation

To launch the basic simulation with default settings:

```bash
roslaunch px100_gazebo px100_gazebo.launch
```

This will start Gazebo with the PX100 robot model and the default controllers.

### Launch with Trajectory Controllers (for MoveIt)

If you plan to use MoveIt for motion planning:

```bash
roslaunch px100_gazebo px100_gazebo.launch use_trajectory_controllers:=true
```

This will load and start the trajectory controllers which are compatible with MoveIt's output.

### Launch with Position Controllers

For direct control of individual joints:

```bash
roslaunch px100_gazebo px100_gazebo.launch use_position_controllers:=true
```

This allows you to control each joint separately via ROS topics.

### Launch with RealSense Camera

To include the simulated RealSense D435 camera:

```bash
roslaunch px100_gazebo px100_gazebo.launch show_realsense_d435:=true
```

This will add a simulated depth camera to the robot model.

## Controlling the Robot in Simulation

### Using Position Controllers

With position controllers, you can command individual joints using ROS topics:

```bash
# Command the waist joint to 0.5 radians
rostopic pub /px100/waist_controller/command std_msgs/Float64 "data: 0.5" -1

# Command the shoulder joint to 0.3 radians
rostopic pub /px100/shoulder_controller/command std_msgs/Float64 "data: 0.3" -1

# Command the elbow joint to -0.5 radians
rostopic pub /px100/elbow_controller/command std_msgs/Float64 "data: -0.5" -1

# Command the wrist joint to 0.7 radians
rostopic pub /px100/wrist_angle_controller/command std_msgs/Float64 "data: 0.7" -1

# Command the gripper to open (distance in meters)
rostopic pub /px100/left_finger_controller/command std_msgs/Float64 "data: 0.037" -1
rostopic pub /px100/right_finger_controller/command std_msgs/Float64 "data: -0.037" -1
```

### Using Trajectory Controllers

With trajectory controllers, you typically send trajectory messages to the `/px100/arm_controller/command` and `/px100/gripper_controller/command` topics. This is normally done through higher-level interfaces like MoveIt, but can also be done manually:

```bash
rostopic pub /px100/arm_controller/command trajectory_msgs/JointTrajectory "{joint_names: ['waist', 'shoulder', 'elbow', 'wrist_angle'], points: [{positions: [0.5, 0.3, -0.5, 0.7], velocities: [0, 0, 0, 0], accelerations: [0, 0, 0, 0], time_from_start: {secs: 1, nsecs: 0}}]}"
```

This command sends a trajectory with a single waypoint, positioning all joints at specific angles after 1 second.

## Visualizing the Simulation

While Gazebo provides its own visualization, you can also use RViz to get a more comprehensive view of the robot's state:

```bash
roslaunch px100_description px100_description.launch use_rviz:=true
```

This will open RViz with the PX100 robot model, which will reflect the state of the robot in Gazebo.

## Troubleshooting Gazebo Simulation

### Common Issues

1. **Gazebo crashes on startup**:
   - Check if you have enough system resources
   - Try setting `SVGA_VGPU10=0` before launching: `export SVGA_VGPU10=0`
   - Update graphics drivers

2. **Controllers not starting**:
   - Check the controller configuration files
   - Look for error messages in the console
   - Ensure controller names match in the launch file and YAML files

3. **Robot model not appearing in Gazebo**:
   - Check for errors in the URDF
   - Ensure the `robot_description` parameter is properly set
   - Verify the spawn_model node is running without errors

4. **Physics issues (robot falling or behaving erratically)**:
   - Check joint limits and inertia values in the URDF
   - Adjust controller gains (PID values) for better stability
   - Reduce simulation step size for more accurate physics

## Conclusion

In this tutorial, we've explored how to set up and run the PX100 robot in Gazebo simulation. We've covered the package structure, controller types, the role of the controller manager, and how to control the robot in simulation. With this knowledge, you can use Gazebo to test and develop robot applications before deploying them to the physical hardware.

In the next tutorial, we'll explore how to integrate the Intel RealSense D435 camera with the PX100 robot for perception tasks.

---

*Previous: [Robot Description](robot-description.md) | Next: [RealSense Integration](realsense-setup.md)*