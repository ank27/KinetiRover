# Getting Started: Basic Setup

*Previous: [Main Page](../index.md) | Next: [Robot Description](robot-description.md)*

## Overview

This guide will walk you through the complete setup process for the KinetiRover project, including operating system installation, ROS Noetic setup, and workspace configuration. By the end of this tutorial, you'll have a fully functioning development environment ready for robotics development.

## Hardware Requirements

- **Development Computer**:
  - Intel Core i5 or equivalent (i7 recommended for simulation)
  - 8GB RAM minimum (16GB recommended)
  - 50GB free disk space
  - NVIDIA GPU recommended for visualization

- **Robot Platform**:
  - NVIDIA Jetson Nano (4GB)
  - PX100 Manipulator
  - Intel RealSense D435 Camera
  - USB Hub (powered recommended)
  - Appropriate power supply

## 1. Ubuntu 20.04 Installation

KinetiRover requires Ubuntu 20.04 LTS (Focal Fossa) as its operating system. If you already have Ubuntu 20.04 installed, you can skip to the next section.

### 1.1 Create Installation Media

1. Download Ubuntu 20.04 LTS from the [official website](https://releases.ubuntu.com/20.04/).
2. Create a bootable USB drive:
   - **On Windows**: Use [Rufus](https://rufus.ie/) or [Etcher](https://www.balena.io/etcher/)
   - **On macOS**: Use [Etcher](https://www.balena.io/etcher/)
   - **On Linux**: Use the `dd` command or [Etcher](https://www.balena.io/etcher/)

### 1.2 Install Ubuntu

1. Insert the bootable USB drive and restart your computer
2. Boot from the USB drive (may require changing boot order in BIOS/UEFI)
3. Select "Install Ubuntu"
4. Follow the installation wizard:
   - Choose your language and keyboard layout
   - Select "Normal installation" and check "Install third-party software"
   - For partitioning, you can choose "Erase disk and install Ubuntu" for a clean install or "Something else" for custom partitioning
   - Set your location, username, and password
5. Complete the installation and reboot

### 1.3 Post-Installation Setup

1. Update your system:
```bash
sudo apt update && sudo apt upgrade -y
```

2. Install essential development tools:
```bash
sudo apt install git build-essential cmake python3-pip -y
```

## 2. ROS Noetic Installation

ROS (Robot Operating System) is the backbone of the KinetiRover project. We'll install ROS Noetic, which is the recommended version for Ubuntu 20.04.

### 2.1 Configure ROS Repositories

1. Setup your computer to accept software from ROS repositories:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

2. Add the ROS repository key:
```bash
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

3. Update your package index:
```bash
sudo apt update
```

### 2.2 Install ROS Noetic

1. Install the full desktop version (recommended):
```bash
sudo apt install ros-noetic-desktop-full -y
```

2. Initialize rosdep (dependency manager):
```bash
sudo rosdep init
rosdep update
```

3. Setup environment variables:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

4. Install build dependencies:
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
```

### 2.3 Test ROS Installation

Verify that ROS is installed correctly by running a ROS master:

1. In one terminal, start the ROS master:
```bash
roscore
```

2. You should see output indicating that the ROS master is running. Keep this terminal open.

3. In a new terminal, check the ROS environment variables:
```bash
printenv | grep ROS
```

4. You should see variables like `ROS_DISTRO`, `ROS_MASTER_URI`, etc.

## 3. Workspace Setup

Next, we'll create a catkin workspace for the KinetiRover project.

### 3.1 Create Catkin Workspace

1. Create the workspace directory structure:
```bash
mkdir -p ~/kinetibot_ws/src
cd ~/kinetibot_ws/
```

2. Initialize the workspace:
```bash
catkin_make
```

3. Source the workspace setup file:
```bash
source ~/kinetibot_ws/devel/setup.bash
```

4. Add the workspace to your .bashrc for automatic sourcing:
```bash
echo "source ~/kinetibot_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.2 Clone the KinetiRover Repository

1. Navigate to the src directory of your workspace:
```bash
cd ~/kinetibot_ws/src
```

2. Clone the KinetiRover repository:
```bash
git clone https://github.com/ank27/KinetiRover.git
```

## 4. Install Dependencies

Now we'll install all the dependencies required for the KinetiRover project.

### 4.1 ROS Package Dependencies

Install ROS packages required for the project:

```bash
sudo apt install ros-noetic-moveit ros-noetic-gazebo-ros-pkgs ros-noetic-joy ros-noetic-joint-state-publisher-gui ros-noetic-tf2-tools ros-noetic-rqt-tf-tree -y
```

### 4.2 PX100 Dependencies

Install the required packages for the PX100 robotic arm:

```bash
sudo apt install ros-noetic-dynamixel-sdk ros-noetic-dynamixel-workbench ros-noetic-dynamixel-workbench-toolbox ros-noetic-joint-trajectory-controller -y
```

### 4.3 RealSense Camera Setup

Install the Intel RealSense SDK and ROS wrapper:

1. Install the RealSense SDK:
```bash
sudo apt install software-properties-common -y
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y
```

2. Verify the installation by running:
```bash
realsense-viewer
```

3. Install the ROS wrapper for RealSense:
```bash
sudo apt install ros-noetic-realsense2-camera ros-noetic-realsense2-description -y
```

### 4.4 Install Supporting Libraries

```bash
sudo apt install libpcl-dev libpcl-ros ros-noetic-pcl-ros python3-pcl python3-sklearn python3-numpy python3-opencv -y
```

## 5. Build the Workspace

With all dependencies installed, let's build the KinetiRover workspace:

```bash
cd ~/kinetibot_ws
catkin_make
```

If you encounter any errors during compilation, make sure all dependencies are correctly installed.

## 6. Verify Installation

Let's verify that everything is set up correctly by running a basic test.

### 6.1 Launch the Robot Description

```bash
roslaunch px100_description px100_description.launch use_rviz:=true
```

This should open RViz with the PX100 robot model loaded. You should be able to see the robot arm model and move the joints using the joint state publisher GUI.

### 6.2 Test Gazebo Simulation

```bash
roslaunch px100_gazebo px100_gazebo.launch
```

Gazebo should open with the PX100 robot model loaded in the simulation environment.

## 7. Setting Up the Jetson Nano

For deploying the project on the KinetiRover robot, you'll need to set up the Jetson Nano.

### 7.1 Flash Jetson Nano

1. Download JetPack from [NVIDIA's website](https://developer.nvidia.com/embedded/jetpack)
2. Follow NVIDIA's instructions to flash the Jetson Nano
3. Complete the initial setup (user account, network, etc.)

### 7.2 Install ROS on Jetson Nano

Follow the same ROS installation steps from Section 2, but on the Jetson Nano.

### 7.3 Deploy KinetiRover Code

1. Create a catkin workspace on the Jetson
2. Clone the KinetiRover repository
3. Build and test the workspace

## Troubleshooting

### Common Issues

1. **Gazebo crashes on startup**:
   - Try setting environment variable: `export SVGA_VGPU10=0`
   - Update graphics drivers

2. **Problems with RealSense**:
   - Check USB connection (use USB 3.0 port)
   - Ensure kernel modules are loaded: `lsmod | grep realsense`

3. **Build errors**:
   - Make sure all dependencies are installed
   - Check for missing packages with: `rosdep check --from-paths src --ignore-src`

## Next Steps

Now that your development environment is set up, you're ready to move on to understanding the robot description files and URDF structure. Continue to the next tutorial: [PX100 Robot Description](robot-description.md).

---

*Previous: [Main Page](../index.md) | Next: [Robot Description](robot-description.md)*