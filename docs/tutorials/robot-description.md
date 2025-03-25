# PX100 Robot Description

*Previous: [Getting Started](getting-started.md) | Next: [Gazebo Simulation](gazebo-simulation.md)*

## Overview

In this tutorial, we'll explore the `px100_description` package, which contains the robot's URDF (Unified Robot Description Format) files, meshes, and launch files. Understanding these components is essential for visualization, simulation, and control of the PX100 manipulator.

## What is URDF?

URDF (Unified Robot Description Format) is an XML format for representing a robot model in ROS. It describes:

- The physical structure of the robot (links)
- The connections between parts (joints)
- Visual appearance (meshes)
- Collision properties
- Inertial properties
- Other kinematic and dynamic properties

The URDF is used by various ROS tools including RViz for visualization, Gazebo for simulation, and MoveIt for motion planning.

## Package Structure

The `px100_description` package follows a standard structure:

```
px100_description/
├── launch/            # Launch files for visualization
├── meshes/            # 3D model files for visualization
│   └── px100_meshes/  # STL files for each robot part
├── rviz/              # RViz configuration files
└── urdf/              # Robot description files
    └── px100.urdf.xacro  # Main robot description file
```

## URDF Structure

The PX100 robot is described using Xacro (XML Macros), which provides templating capabilities for URDF files. This makes the robot description more modular and easier to maintain.

### Main Components

1. **Links**: Physical rigid bodies of the robot
   - `base_link`
   - `shoulder_link`
   - `upper_arm_link`
   - `forearm_link`
   - `wrist_link`
   - `gripper_link`
   - `left_finger_link` and `right_finger_link`

2. **Joints**: Connections between links that define how they can move
   - `waist`: Connects base to shoulder (revolute)
   - `shoulder`: Connects shoulder to upper arm (revolute)
   - `elbow`: Connects upper arm to forearm (revolute)
   - `wrist_angle`: Connects forearm to wrist (revolute)
   - `gripper`: Actuates the gripper
   - `left_finger` and `right_finger`: Control the gripper fingers

3. **Visual and Collision Elements**: Define how the robot appears and interacts physically
   - STL mesh files for visual representation
   - Simplified collision geometries for efficient collision checking

### Key URDF Features

Let's examine some key sections from the PX100 URDF:

```xml
<!-- Base link definition -->
<link name="$(arg robot_name)/$(arg base_link_frame)">
  <visual>
    <origin rpy="0 0 ${pi/2}" xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://px100_description/meshes/px100_meshes/px100_1_base.stl" scale="0.001 0.001 0.001"/>
    </geometry>
    <material name="px100_black"/>
  </visual>
  <collision>
    <origin rpy="0 0 ${pi/2}" xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://px100_description/meshes/px100_meshes/px100_1_base.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </collision>
  <inertial>
    <origin rpy="0 0 ${pi/2}" xyz="-0.0332053000 0.0008915770 0.0211913000"/>
    <mass value="0.395887" />
    <inertia ixx="0.0010650000" iyy="0.0003332000" izz="0.0012080000" ixy="-0.0000130300" ixz="0.0000018614" iyz="0.0000409200" />
  </inertial>
</link>

<!-- Waist joint connecting base to shoulder -->
<joint name="waist" type="revolute">
  <axis xyz="0 0 1"/>
  <limit effort="1" lower="${-pi + pi_offset}" upper="${pi - pi_offset}" velocity="${pi}"/>
  <origin rpy="0 0 0" xyz="0 0 0.05085"/>
  <parent link="$(arg robot_name)/$(arg base_link_frame)"/>
  <child link="$(arg robot_name)/shoulder_link"/>
  <dynamics friction="0.1"/>
</joint>
```

This code defines:
1. The base link with its visual, collision, and inertial properties
2. The waist joint that connects the base to the shoulder with rotation limits

## Launch Files

The package includes launch files to visualize and test the robot model. The main launch file is `px100_description.launch`, which loads the URDF and starts visualization tools.

```xml
<launch>
  <arg name="robot_model"                       default="px100"/>
  <arg name="robot_name"                        default="$(arg robot_model)"/>
  <arg name="base_link_frame"                   default="base_link"/>
  <arg name="show_ar_tag"                       default="false"/>
  <arg name="show_gripper_bar"                  default="true"/>
  <arg name="show_gripper_fingers"              default="true"/>
  <arg name="use_world_frame"                   default="true"/>
  <arg name="external_urdf_loc"                 default=""/>
  <arg name="use_rviz"                          default="true"/>
  <arg name="load_gazebo_configs"               default="false"/>
  <arg name="use_joint_pub"                     default="false"/>
  <arg name="use_joint_pub_gui"                 default="false"/>
  <arg name="rate"                              default="10"/>
  
  <!-- Load robot description to parameter server -->
  <param name="$(arg robot_name)/robot_description" command="xacro $(arg model)" />

  <!-- Start joint state publisher -->
  <node if="$(arg use_joint_pub)"
    name="joint_state_publisher"
    pkg="joint_state_publisher"
    type="joint_state_publisher"
    ns="$(arg robot_name)">
    <param name="rate" value="$(arg rate)"/>
  </node>

  <!-- Start robot state publisher -->
  <node
    name="robot_state_publisher"
    pkg="robot_state_publisher"
    type="robot_state_publisher"
    ns="$(arg robot_name)">
  </node>

  <!-- Launch RViz if requested -->
  <node if="$(arg use_rviz)"
    name="rviz"
    pkg="rviz"
    type="rviz"
    args="-d $(arg rvizconfig)"
    ns="$(arg robot_name)"/>
</launch>
```

## Robot State Publisher and Joint State Publisher

### Robot State Publisher

The `robot_state_publisher` package is a crucial component in ROS robotics. It:

1. **Publishes TF Transforms**: Converts joint positions into TF transforms
2. **Creates the Robot Tree**: Builds a tree structure representing the robot's kinematic chain
3. **Enables 3D Visualization**: Allows tools like RViz to display the robot model correctly
4. **Supports Perception and Control**: Provides the coordinate frames needed for perception and control algorithms

#### How it Works:

1. Reads the URDF from the parameter server
2. Subscribes to `/joint_states` topic
3. Calculates the forward kinematics based on joint positions
4. Publishes the resulting transforms to the `/tf` topic

### Joint State Publisher

The `joint_state_publisher` package:

1. **Generates Joint States**: Creates joint positions for testing and visualization
2. **Simulates Joint Movement**: In the absence of actual hardware or controllers
3. **Has GUI Option**: Can launch with a slider interface for manual joint control
4. **Publishes to `/joint_states`**: Sends joint positions that the robot_state_publisher uses

#### How it Works:

1. Reads the URDF from the parameter server
2. Identifies all movable joints and their limits
3. Generates random or user-defined joint positions within limits
4. Publishes these positions to the `/joint_states` topic

### Use Cases

#### Development and Testing

During development, these tools allow you to:
- Verify that the URDF is correct
- Test the robot's range of motion
- Visualize the robot before working with hardware
- Validate transformation chains

#### Integration with Hardware

When working with real hardware:
- The `joint_state_publisher` is replaced by actual hardware controllers
- These controllers publish the real joint states of the robot
- The `robot_state_publisher` continues to work with these real joint states

#### Simulation

In Gazebo:
- Simulated controllers publish joint states
- `robot_state_publisher` uses these to maintain the TF tree
- This creates a consistent interface between simulation and real hardware

## Visualizing the Robot

### Using RViz

To visualize the PX100 in RViz:

```bash
roslaunch px100_description px100_description.launch use_rviz:=true use_joint_pub_gui:=true
```

This will open RViz with the robot model and a joint control GUI. You can:
- Move the sliders to change joint positions
- See how the robot would look in different configurations
- Verify that joint limits are correct

### Understanding the TF Tree

You can visualize the transformation tree using the `tf_tree` tool:

```bash
rosrun tf2_tools view_frames.py
```

This generates a PDF showing the relationship between all the coordinate frames of the robot.

## Joint Limits and Specifications

The PX100 has the following joint limits, defined in the URDF:

| Joint | Min (degrees) | Max (degrees) | Type |
|-------|---------------|---------------|------|
| Waist | -180 | 180 | Revolute |
| Shoulder | -111 | 107 | Revolute |
| Elbow | -121 | 92 | Revolute |
| Wrist Angle | -100 | 123 | Revolute |
| Gripper | 0 | 37mm (linear) | Prismatic |

These limits are crucial for:
- Ensuring safe operation
- Proper motion planning
- Preventing self-collisions

## Mesh Files

The PX100 uses STL mesh files for visual representation. These files are located in the `meshes/px100_meshes/` directory and include:

- `px100_1_base.stl`: The robot's base
- `px100_2_shoulder.stl`: The shoulder link
- `px100_3_upper_arm.stl`: The upper arm
- `px100_4_forearm.stl`: The forearm
- `px100_5_gripper.stl`: The wrist and gripper mount
- `px100_6_gripper_prop.stl`: Gripper components
- `px100_7_gripper_bar.stl`: Gripper structure
- `px100_8_gripper_finger.stl`: Gripper fingers

The meshes are scaled in the URDF (using `scale="0.001 0.001 0.001"`) because they're designed in millimeters, but ROS uses meters.

## Practical Exercise: Modifying the URDF

Let's do a practical exercise to modify the URDF:

1. Make a copy of the URDF file:
   ```bash
   cp ~/kinetibot_ws/src/KinetiRover/px100_description/urdf/px100.urdf.xacro ~/kinetibot_ws/src/KinetiRover/px100_description/urdf/px100_modified.urdf.xacro
   ```

2. Edit the file to change a joint limit, for example, increase the shoulder range:
   ```bash
   sed -i 's/limit effort="2" lower="${radians(-111)}" upper="${radians(107)}" velocity="${pi}"/limit effort="2" lower="${radians(-120)}" upper="${radians(120)}" velocity="${pi}"/g' ~/kinetibot_ws/src/KinetiRover/px100_description/urdf/px100_modified.urdf.xacro
   ```

3. Create a new launch file to use your modified URDF:
   ```bash
   cp ~/kinetibot_ws/src/KinetiRover/px100_description/launch/px100_description.launch ~/kinetibot_ws/src/KinetiRover/px100_description/launch/px100_modified.launch
   ```

4. Edit the launch file to use your modified URDF:
   ```bash
   sed -i 's/px100.urdf.xacro/px100_modified.urdf.xacro/g' ~/kinetibot_ws/src/KinetiRover/px100_description/launch/px100_modified.launch
   ```

5. Launch RViz with your modified robot:
   ```bash
   roslaunch px100_description px100_modified.launch use_rviz:=true use_joint_pub_gui:=true
   ```

6. Test if your joint limits have changed using the joint state publisher GUI

## Troubleshooting

### Common Issues

1. **Robot appears in the wrong position/orientation**:
   - Check the transformation tree with `rosrun tf2_tools view_frames.py`
   - Verify that all joints have the correct parent-child relationships

2. **Missing meshes in RViz**:
   - Make sure all mesh paths are correct in the URDF
   - Check if ROS_PACKAGE_PATH includes the px100_description package

3. **Joint limits not working**:
   - Verify the joint limit values in the URDF
   - Check if the joint_state_publisher is respecting these limits

4. **Mismatched transforms**:
   - Make sure all origins (rpy and xyz) are specified correctly
   - Ensure consistency between visual, collision, and inertial origins

## Next Steps

Now that you understand the PX100 robot description, you're ready to move on to simulating the robot in Gazebo. Continue to the next tutorial: [Gazebo Simulation](gazebo-simulation.md).

---

*Previous: [Getting Started](getting-started.md) | Next: [Gazebo Simulation](gazebo-simulation.md)*