# KinetiRover Project Setup Tutorial

Welcome to the KinetiRover project documentation! This guide will walk you through setting up your development environment and understanding the key components of our robotic system.

## Hardware Overview

The KinetiRover project uses the PincherX-100 (PX100) robotic arm from Interbotix. This is a 4-DOF manipulator with the following specifications:

- **Degrees of Freedom**: 4
- **Maximum Reach**: 300mm
- **Total Span**: 600mm
- **Working Payload**: 50g (recommended at 50% arm extension)
- **Repeatability**: 5mm
- **Accuracy**: 8mm
- **Total Servos**: 5 (4 for arm joints, 1 for gripper)

### Key Components
1. **PX100 Robotic Arm**
   - Uses DYNAMIXEL X-Series Smart Servo Motors (XL430-W250)
   - Features high-resolution positioning (4096 positions)
   - Includes temperature monitoring and positional feedback
   - Controlled via DYNAMIXEL U2D2 interface

2. **Intel RealSense D435 Camera**
   - Provides RGB-D (color + depth) imaging
   - Used for perception and object detection
   - Enables 3D mapping of the environment

3. **DYNAMIXEL Motors**
   - Used for precise joint control
   - Provides feedback for position, velocity, and load
   - Programmable PID parameters for motion control

## Software Requirements

### Operating System
- Ubuntu 20.04 LTS (Focal Fossa)

### ROS Installation
1. Install ROS Noetic:
```bash
# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update package index
sudo apt update

# Install ROS Noetic Desktop Full
sudo apt install ros-noetic-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

# Environment setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

2. Install additional dependencies:
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### Simulation Tools
1. **Gazebo** (included with ROS Desktop Full)
   - Used for physics simulation
   - Enables testing without physical hardware
   - Provides realistic sensor simulation

2. **RViz**
   - 3D visualization tool
   - Displays robot state and sensor data
   - Helps in monitoring and debugging

## Project Setup

1. Create a ROS workspace:
```bash
mkdir -p ~/kinetibot_ws/src
cd ~/kinetibot_ws
catkin_make
```

2. Clone the repository:
```bash
cd ~/kinetibot_ws/src
git clone https://github.com/ank27/KinetiRover.git
```

3. Install dependencies:
```bash
cd ~/kinetibot_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace:
```bash
catkin_make
source devel/setup.bash
```

## Testing the Setup

1. Launch the Gazebo simulation:
```bash
roslaunch px100_gazebo px100_gazebo.launch
```

2. Launch RViz visualization:
```bash
roslaunch px100_description px100_description.launch use_rviz:=true
```

## Next Steps

1. Review the [PX100 Documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/) for detailed hardware specifications
2. Explore the source code in the `src` directory
3. Try running the example scripts in the `examples` directory

## Contributing

1. Create a new branch from master:
```bash
git checkout -b feature/your-feature-name
```

2. Make your changes and commit them:
```bash
git add .
git commit -m "Description of changes"
```

3. Push to GitHub and create a pull request:
```bash
git push origin feature/your-feature-name
```

## Support

For questions or issues:
1. Open an issue on GitHub
2. Check existing documentation
3. Contact the development team

---
This documentation is part of the KinetiRover project. For more detailed information, please visit our [GitHub repository](https://github.com/ank27/KinetiRover).