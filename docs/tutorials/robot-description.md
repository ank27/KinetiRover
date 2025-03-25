# PX100 Robot Description

*Previous: [Getting Started](getting-started.md) | Next: [Gazebo Simulation](gazebo-simulation.md)*

## Overview

This tutorial explains the PX100 robot description package, which contains the URDF (Unified Robot Description Format) files, mesh models, and launch files necessary to visualize and simulate the robot. Understanding this package is essential for working with the PX100 manipulator in both simulation and real-world applications.

## Package Structure

The `px100_description` package contains all the files needed to describe the robot's physical structure, appearance, and joint limits. Let's explore its organization:

```
px100_description/
├── launch/
│   ├── px100_description.launch
│   └── px100_upload.launch
├── meshes/
│   ├── px100_meshes/
│   │   ├── px100_1_base.stl
│   │   ├── px100_2_shoulder.stl
│   │   ├── px100_3_upper_arm.stl
│   │   ├── px100_4_forearm.stl
│   │   ├── px100_5_gripper.stl
│   │   ├── px100_6_gripper_prop.stl
│   │   ├── px100_7_gripper_bar.stl
│   │   └── px100_8_gripper_finger.stl
│   └── px100_black.png
├── rviz/
│   └── px100_description.rviz
├── urdf/
│   ├── px100.urdf.xacro
│   └── px100_configs.urdf.xacro
├── CMakeLists.txt
└── package.xml
```

## URDF Structure

Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe the physical properties of a robot. For our PX100 manipulator, we use XACRO (XML Macros) to make the URDF more modular and maintainable.

### Main URDF Components

1. **Links**: Represent rigid body components of the robot
2. **Joints**: Define how links connect and move relative to each other
3. **Visual Elements**: Define how the robot appears (using meshes or primitive shapes)
4. **Collision Elements**: Define simplified geometries for collision checking
5. **Inertial Properties**: Define mass, center of mass, and inertia matrix for each link

### PX100 URDF Files

The main URDF file for the PX100 is `px100.urdf.xacro`. This file defines the robot's structure with the following key components:

#### Links

- `base_link`: The base of the robot
- `shoulder_link`: The shoulder joint housing
- `upper_arm_link`: The upper arm segment
- `forearm_link`: The forearm segment
- `wrist_link`: The wrist joint housing
- `gripper_link`: The gripper assembly
- `left_finger_link` and `right_finger_link`: The gripper fingers

#### Joints

The PX100 has 4 degrees of freedom (DOF) in its arm plus a gripper:

1. **Waist** (joint between base_link and shoulder_link): Rotating base joint
2. **Shoulder** (joint between shoulder_link and upper_arm_link): Shoulder joint
3. **Elbow** (joint between upper_arm_link and forearm_link): Elbow joint
4. **Wrist Angle** (joint between forearm_link and gripper_link): Wrist pitch joint
5. **Gripper** (joint for the fingers): Gripper open/close

## Joint Limits and Specifications

Each joint in the PX100 has specific limits that define its range of motion. These limits are essential for both safety and proper operation. Here's a breakdown of the joint limits:

| Joint       | Min Angle (degrees) | Max Angle (degrees) | Servo ID |
|-------------|--------------------|--------------------|---------|
| Waist       | -180               | 180                | 1       |
| Shoulder    | -111               | 107                | 2       |
| Elbow       | -121               | 92                 | 3       |
| Wrist Angle | -100               | 123                | 4       |
| Gripper     | 30mm               | 74mm (opening)     | 5       |

### Joint Limits in URDF

In the URDF file, joint limits are defined within each joint tag. For example:

```xml
<joint name="shoulder" type="revolute">
  <axis xyz="0 1 0"/>
  <limit effort="2" lower="${radians(-111)}" upper="${radians(107)}" velocity="${pi}"/>
  <origin rpy="0 0 0" xyz="0 0 0.04225"/>
  <parent link="$(arg robot_name)/shoulder_link"/>
  <child link="$(arg robot_name)/upper_arm_link"/>
  <dynamics friction="0.1"/>
</joint>
```

The attributes of the `<limit>` tag define:

- **effort**: Maximum torque (in Nm) that can be applied
- **lower**: Lower joint limit (in radians)
- **upper**: Upper joint limit (in radians)
- **velocity**: Maximum joint velocity (in rad/s)

## Mesh Files

The `meshes` folder contains 3D models (STL files) for each link of the robot. These files define the visual appearance of the robot and are referenced in the URDF file. The meshes are used by both RViz and Gazebo for visualization.

For the PX100, we use detailed mesh files to represent the actual geometry of the robot components, making the visualization accurate to the real robot.

## Launch Files

The `launch` folder contains ROS launch files that set up the robot model for visualization and simulation.

### px100_description.launch

This is the primary launch file for loading and visualizing the robot model. Let's examine its key components:

```xml
<launch>
  <arg name="robot_model" default="px100"/>
  <arg name="robot_name" default="$(arg robot_model)"/>
  <arg name="base_link_frame" default="base_link"/>
  <arg name="show_ar_tag" default="false"/>
  <arg name="show_gripper_bar" default="true"/>
  <arg name="show_gripper_fingers" default="true"/>
  <arg name="use_world_frame" default="true"/>
  <arg name="external_urdf_loc" default=""/>
  <arg name="use_rviz" default="true"/>
  <arg name="load_gazebo_configs" default="false"/>
  <arg name="use_joint_pub" default="false"/>
  <arg name="use_joint_pub_gui" default="false"/>
  <arg name="rvizconfig" default="$(find px100_description)/rviz/px100_description.rviz" />

  <param name="$(arg robot_name)/robot_description" command="xacro $(arg model)" />

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" ns="$(arg robot_name)"/>

  <node if="$(arg use_joint_pub)" name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" ns="$(arg robot_name)"/>

  <node if="$(arg use_joint_pub_gui)" name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" ns="$(arg robot_name)"/>

  <node if="$(arg use_rviz)" name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" ns="$(arg robot_name)"/>
</launch>
```

This launch file:
1. Defines arguments for customizing the robot's configuration
2. Loads the robot description onto the parameter server
3. Starts the `robot_state_publisher`
4. Optionally starts the `joint_state_publisher` or `joint_state_publisher_gui`
5. Optionally starts RViz with a pre-configured setup

## Robot State Publisher and Joint State Publisher

These two nodes are fundamental to working with robot models in ROS:

### Robot State Publisher

**Purpose**: Publishes the state of the robot to the TF (Transform) tree.

**Function**: 
- Takes the robot description (URDF) from the parameter server
- Listens to joint states on the `/joint_states` topic
- Calculates the forward kinematics of the robot
- Publishes the positions of all links to the TF tree

**Use Case**: 
- Necessary for visualization tools like RViz to display the robot model
- Required for planning and control algorithms that need to know the current state of the robot
- Used by perception systems to transform sensor data into the robot's coordinate frame

### Joint State Publisher

**Purpose**: Publishes joint states for a robot when joint states are not being published by another source.

**Function**:
- Reads the robot description to identify all joints
- Publishes joint positions on the `/joint_states` topic
- The GUI version provides sliders to manually manipulate joint positions

**Use Case**:
- Testing and visualization when real robot hardware or controllers are not running
- Useful for debugging and development
- The GUI version allows interactive manipulation of the robot model

## Running the PX100 Description

To visualize the PX100 robot in RViz, use the following command:

```bash
roslaunch px100_description px100_description.launch use_rviz:=true use_joint_pub_gui:=true
```

This command will:
1. Load the robot model onto the parameter server
2. Start the robot state publisher
3. Launch the joint state publisher with GUI
4. Open RViz with a pre-configured view of the robot

You should see the PX100 robot model in RViz, and you can use the joint state publisher GUI to move the joints interactively.

### Command Arguments

The launch file supports several arguments to customize the visualization:

- `use_rviz`: Set to `true` to open RViz with the robot model
- `use_joint_pub_gui`: Set to `true` to launch the joint state publisher GUI
- `show_gripper_fingers`: Set to `true` to display the gripper fingers
- `show_ar_tag`: Set to `true` to display an AR tag on the robot (if applicable)

Example with customizations:

```bash
roslaunch px100_description px100_description.launch use_rviz:=true use_joint_pub_gui:=true show_ar_tag:=true
```

## Understanding TF (Transform) Tree

When running the robot description, ROS builds a transform tree that represents the spatial relationships between all links in the robot. You can visualize this tree using:

```bash
rosrun tf2_tools view_frames.py
```

This will generate a PDF file showing the structure of the transform tree.

Alternatively, you can visualize the TF tree in RViz by adding a TF display.

## Conclusion

Understanding the robot description package is fundamental to working with the PX100 manipulator. The URDF defines the physical properties and constraints of the robot, which are essential for visualization, simulation, planning, and control.

In the next tutorial, we'll explore how to use the robot description in Gazebo to create a physics-based simulation of the PX100 manipulator.

---

*Previous: [Getting Started](getting-started.md) | Next: [Gazebo Simulation](gazebo-simulation.md)*