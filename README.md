# KinetiRover

This repository contains the codebase for a robotic manipulator system based on the [PX100 robot from Trossen Robotics](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/px100.html). The system integrates both simulation and real hardware capabilities using ROS (Robot Operating System).

## Documentation

**[📚 View the complete documentation and tutorials](https://ank27.github.io/KinetiRover)**

Visit our GitHub Pages site for comprehensive guides on:
- System setup and installation
- Robot description and configuration
- Gazebo simulation
- RealSense camera integration
- Perception pipeline
- MoveIt integration

## System Overview

The PX100 is a 4-DOF robotic manipulator with the following specifications:
- 4 degrees of freedom
- Dynamixel servo motors
- Intel RealSense stereo camera for perception
- Full integration with MoveIt for motion planning
- Gazebo simulation support

## Repository Structure

```
mr_kinetik/
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

## Key Components

### 1. Hardware Components
- **Robot**: PX100 4-DOF manipulator from Trossen Robotics
- **Motors**: Dynamixel servos for precise control
- **Sensor**: Intel RealSense stereo camera for depth sensing and computer vision

### 2. Software Framework
- **ROS**: Primary robotics middleware
- **MoveIt**: Motion planning and manipulation
- **Gazebo**: Physics simulation
- **Programming Languages**: Python, C++, XML

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
git clone --recursive https://github.com/your-org/mr_kinetik.git
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

## Development

### Contributing
1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License
[Insert License Information]

## Contact
[Insert Contact Information]
