# KinetiRover

A ROS-based mobile manipulation platform combining a 4-wheel drive rover with PX100 robotic manipulator. Features integrated motion planning, simulation, and perception using Jetson Nano, Dynamixel motors, and RealSense camera.

## System Overview

### Hardware Components
- 4-wheel drive mobile base
- PX100 4-DOF manipulator from [Trossen Robotics](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/px100.html)
- Dynamixel servo motors
- Intel RealSense stereo camera
- Jetson Nano computing unit

### Software Framework
- ROS (Robot Operating System)
- MoveIt for motion planning
- Gazebo for simulation
- RealSense SDK for perception

## Repository Structure

```
KinetiRover/
├── src/
│   ├── arm/
│   │   ├── px100_core/         # Core control and hardware interface
│   │   ├── px100_description/  # URDF models and meshes
│   │   ├── px100_gazebo/      # Gazebo simulation files
│   │   ├── px100_moveit/      # MoveIt configuration
│   │   ├── px100_moveit_impl/ # MoveIt implementation code
│   │   └── px100_perception/  # RealSense camera integration
│   └── vendor/                # Third-party dependencies
```

## Quick Start

1. **Installation Prerequisites**
```bash
# Install ROS (instructions for ROS Noetic)
sudo apt install ros-noetic-desktop-full

# Install additional dependencies
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-gazebo-ros-pkgs
```

2. **Clone the Repository**
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recursive https://github.com/ank27/KinetiRover.git
```

3. **Build the Workspace**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Launch Files

### Simulation
```bash
# Launch Gazebo simulation
roslaunch px100_gazebo px100_gazebo.launch

# Launch MoveIt with Gazebo
roslaunch px100_moveit_impl px100_moveit_gazebo.launch
```

### Real Hardware
```bash
# Launch hardware interface
roslaunch px100_core px100_core.launch

# Launch MoveIt with real hardware
roslaunch px100_moveit_impl px100_moveit_hardware.launch
```

## Contributing
1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request