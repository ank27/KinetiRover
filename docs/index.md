# KinetiRover Project Documentation

Welcome to the KinetiRover documentation! This series of tutorials will guide you through setting up and working with the KinetiRover robotic system, which uses the PX100 robotic arm from Interbotix.

## Tutorial Series

1. [Getting Started: Basic Setup](tutorials/getting-started.md)
   - Ubuntu 20.04 setup
   - ROS Noetic installation
   - Workspace configuration
   - Dependencies installation

2. [PX100 Robot Description](tutorials/robot-description.md)
   - Understanding URDF structure
   - Robot model configuration
   - Joint limits and specifications
   - Testing robot visualization

3. [Gazebo Simulation](tutorials/gazebo-simulation.md)
   - Setting up Gazebo environment
   - Loading robot in simulator
   - Basic movement testing
   - Joint control interface

4. [RealSense Integration](tutorials/realsense-setup.md)
   - RealSense D435 hardware setup
   - Camera drivers installation
   - Point cloud visualization
   - Camera calibration

5. [Perception Pipeline](tutorials/perception-pipeline.md)
   - Point cloud processing
   - Object detection setup
   - Segmentation techniques
   - Workspace monitoring

6. [MoveIt Integration](tutorials/moveit-setup.md)
   - MoveIt configuration
   - Motion planning
   - Trajectory execution
   - Pick and place tasks

## Hardware Specifications

The KinetiRover uses the PX100 robotic arm with the following specifications:

- **Degrees of Freedom**: 4
- **Maximum Reach**: 300mm
- **Working Payload**: 50g
- **Total Servos**: 5 (4 for arm joints, 1 for gripper)

### System Components

1. **PX100 Robotic Arm**
   - DYNAMIXEL X-Series Smart Servo Motors
   - High-resolution positioning (4096 positions)
   - Built-in temperature and position feedback

2. **Intel RealSense D435**
   - RGB-D camera for perception
   - Depth sensing capabilities
   - 1280x720 resolution

## Installation Requirements

- Ubuntu 20.04 LTS
- ROS Noetic
- Gazebo 11
- MoveIt Framework
- RealSense SDK 2.0

## Contributing

This project is open for contributions. Please follow our [contribution guidelines](contributing.md) when submitting changes.

## Support

If you encounter any issues:
1. Check our [troubleshooting guide](troubleshooting.md)
2. Open an issue on GitHub
3. Contact the development team

---
Next Tutorial: [Getting Started: Basic Setup](tutorials/getting-started.md)