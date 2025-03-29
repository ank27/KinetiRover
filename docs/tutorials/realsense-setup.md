# RealSense D435 Integration

*Previous: [Gazebo Simulation](gazebo-simulation.md) | Next: [Perception Pipeline](perception-pipeline.md)*

## Overview

This tutorial covers the integration of the Intel RealSense D435 depth camera with the KinetiRover project. The RealSense D435 is a crucial component of our robotic system, providing depth perception capabilities essential for object detection, manipulation, and environmental mapping.

## Why RealSense D435?

The Intel RealSense D435 camera was selected for the KinetiRover project for several key reasons:

1. **Depth Perception**: The D435 provides accurate depth information up to 10 meters, making it perfect for robotic navigation and object detection.

2. **Wide Field of View**: Its global shutter with 85° × 58° FOV enables good visibility of the robot's environment.

3. **RGB + Depth**: It provides both color and depth information, which is essential for object recognition and segmentation.

4. **ROS Integration**: Excellent support for ROS through the realsense-ros package.

5. **Compact Size**: The small form factor allows for easy mounting on the KinetiRover platform.

6. **Robust SDK**: Intel provides a comprehensive SDK that makes interfacing with the camera straightforward.

7. **Active Stereo Technology**: Uses IR projector for improved performance in textureless environments.

## Hardware Setup

### Physical Installation

1. **Mounting Position**: The D435 camera should be mounted at the front of the robot, with a clear view of the workspace. For the PX100 manipulator, we typically mount it on a separate stand or on the base of the robot, positioned to view the area where the gripper will operate.

2. **USB Connection**: Connect the camera to your computer or Jetson Nano using a USB 3.0 cable. The D435 requires USB 3.0 for full functionality.

3. **Power Considerations**: When using with Jetson Nano, ensure you have adequate power supply, as the camera draws additional power.

### Camera Specifications

- **Depth Technology**: Active IR Stereo
- **Depth FOV**: 85° × 58° (H × V)
- **RGB Resolution**: Up to 1920 × 1080 at 30 fps
- **Depth Resolution**: Up to 1280 × 720 at 90 fps
- **Minimum Depth Distance**: ~0.2m
- **Maximum Range**: Up to 10m (environment dependent)
- **Dimensions**: 90 mm × 25 mm × 25 mm

## Software Installation

### 1. Install RealSense SDK

The RealSense SDK 2.0 provides the drivers and libraries needed to interface with the camera. We install it from Intel's official repository:

```bash
# Add Intel's repository
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# Install the SDK
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y
```

### 2. Test Basic Camera Functionality

After installing the SDK, you can test if your camera is working correctly using Intel's utility:

```bash
realsense-viewer
```

This will open Intel's RealSense Viewer application. You should see your camera listed and be able to view both color and depth streams.

### 3. Install ROS Package from Vendor Source

While the RealSense ROS package is available in the ROS repositories, we prefer to use the source version from Intel's GitHub repository. This approach offers several advantages:

- **Access to Latest Features**: The source version often includes the newest features and bug fixes.
- **Specific Configurations**: We can customize the build for our specific requirements.
- **Alignment with SDK Version**: Ensures compatibility with our installed SDK version.
- **Wrapper Optimizations**: Access to optimized wrappers for our specific hardware.

Here's how to install the RealSense ROS package from source:

```bash
# Navigate to your catkin workspace source directory
cd ~/workspace/src

# Clone the RealSense ROS wrapper
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/

# Checkout the branch that matches your ROS version (Noetic in our case)
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+$" | tail -1`

# Install dependencies
cd ~/workspace
sudo apt-get install ros-noetic-ddynamic-reconfigure -y
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install

# Source the workspace
source devel/setup.bash
```

## Running the RealSense Camera in ROS

### Basic Launch

To start the RealSense camera node with default settings:

```bash
roslaunch realsense2_camera rs_camera.launch
```

This command launches the camera with standard parameters. You should see output indicating that the camera node has started successfully.

### Advanced Launch Options

The RealSense ROS package offers many configuration options. Here's a more comprehensive launch command for the KinetiRover project:

```bash
roslaunch realsense2_camera rs_camera.launch \
    align_depth:=true \
    filters:=pointcloud \
    depth_width:=640 \
    depth_height:=480 \
    depth_fps:=30 \
    color_width:=640 \
    color_height:=480 \
    color_fps:=30
```

This configures:
- Alignment of depth and color frames
- Generation of point cloud data
- Resolution and frame rate for both depth and color streams

### Topics Published

With the camera running, several ROS topics are published:

```bash
# List all camera topics
rostopic list | grep camera
```

Key topics include:
- `/camera/color/image_raw` - RGB camera feed
- `/camera/depth/image_rect_raw` - Depth image
- `/camera/depth/color/points` - Aligned point cloud
- `/camera/aligned_depth_to_color/image_raw` - Depth aligned to color image
- `/camera/extrinsics/depth_to_color` - Extrinsic calibration data

## Visualizing Point Clouds

One of the most important capabilities of the RealSense D435 is generating point clouds - 3D representations of the environment that combine color and depth information.

### Using RViz

1. Launch RViz:
```bash
rosrun rviz rviz
```

2. Configure RViz to display the point cloud:
   - Click "Add" in the displays panel
   - Select "PointCloud2" from the list
   - Set the Topic to `/camera/depth/color/points`
   - Set "Fixed Frame" to `camera_link` or your preferred frame

3. Adjust visualization settings:
   - You can modify the Size (point size) and Style settings to improve visualization
   - Under "Color Transformer", select "RGB8" to view the colored point cloud

### Custom Point Cloud Visualization

For more advanced visualization, you can also use a custom launch file that integrates the camera with RViz presets:

```bash
# Create a custom launch file in your package
roscd kinetirover_perception
touch launch/view_pointcloud.launch
```

Add the following content to the launch file:

```xml
<launch>
  <!-- Start the RealSense camera -->
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="align_depth" value="true" />
    <arg name="filters" value="pointcloud" />
  </include>
  
  <!-- Start RViz with a custom configuration -->
  <node type="rviz" name="rviz" pkg="rviz" args="-d $(find kinetirover_perception)/rviz/pointcloud_viewer.rviz" />
</launch>
```

Create a custom RViz configuration that automatically includes the point cloud display:

```bash
rosrun rviz rviz -d $(find kinetirover_perception)/rviz/pointcloud_viewer.rviz
```

Save the configuration to your package's rviz folder.

## Camera Calibration

Although the RealSense D435 comes factory-calibrated, additional calibration may be necessary for precise robotic applications, especially when integrating with the PX100 manipulator.

### Types of Calibration

1. **Intrinsic Calibration**: Relates to the internal camera parameters like focal length, optical center, and distortion. This is typically handled by the factory calibration.

2. **Extrinsic Calibration**: Defines the transformation between the camera and other frames of reference (e.g., the robot base).

### Camera-Robot Calibration

To use the camera effectively with the PX100 manipulator, we need to determine the transform between the camera frame and the robot's base frame.

1. **Using the AR Tag Method**:

   First, install the required packages:
   ```bash
   sudo apt-get install ros-noetic-aruco-ros ros-noetic-easy-handeye -y
   ```

   Attach an AR tag to the robot's end-effector and run:
   ```bash
   roslaunch easy_handeye calibrate.launch \
       eye_on_base:=true \
       tracking_base_frame:=camera_link \
       tracking_marker_frame:=ar_marker_0 \
       robot_base_frame:=px100/base_link \
       robot_effector_frame:=px100/ee_gripper_link
   ```

   Follow the on-screen instructions to move the robot to different positions and capture samples.

2. **Manual Calibration**:

   For simpler setups, you can manually measure the position of the camera relative to the robot base and define the transform in a static TF publisher:

   ```xml
   <node pkg="tf" type="static_transform_publisher" name="camera_link_broadcaster"
       args="0.2 0 0.3 0 0 0 px100/base_link camera_link 100" />
   ```

   Adjust the parameters (position and orientation) based on your measurements.

### Testing Camera Calibration

After calibration, you can verify the results by:

1. Using RViz to visualize both the robot model and point cloud together
2. Running a simple pick operation to test if the robot can accurately move to positions determined from the camera data

```bash
# Launch the robot description and camera
roslaunch px100_description px100_description.launch use_rviz:=true
roslaunch realsense2_camera rs_camera.launch

# Add the calibration transform
roslaunch kinetirover_perception camera_calibration.launch
```

## Integrated Camera Launch

For convenience, we can create a single launch file that starts both the camera and the necessary calibration information:

```xml
<launch>
  <!-- Start the RealSense camera -->
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="align_depth" value="true" />
    <arg name="filters" value="pointcloud" />
  </include>
  
  <!-- Load camera calibration -->
  <node pkg="tf" type="static_transform_publisher" name="camera_link_broadcaster"
      args="0.2 0 0.3 0 0 0 px100/base_link camera_link 100" />
      
  <!-- Start RViz with configuration -->
  <node type="rviz" name="rviz" pkg="rviz" 
      args="-d $(find kinetirover_perception)/rviz/camera_robot.rviz" />
</launch>
```

Save this to `kinetirover_perception/launch/camera_robot.launch` and run with:

```bash
roslaunch kinetirover_perception camera_robot.launch
```

## Common Issues and Troubleshooting

### USB Connection Problems

- **Symptom**: Camera not detected or frequent disconnections
- **Solution**: 
  - Ensure you're using a USB 3.0 port and cable
  - Check USB power settings: `sudo sh -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend'`
  - Try a powered USB hub if power issues persist

### Point Cloud Visibility

- **Symptom**: Point cloud not visible in RViz
- **Solution**:
  - Verify topic is publishing: `rostopic echo /camera/depth/color/points -n 1`
  - Check frame settings in RViz
  - Ensure `filters:=pointcloud` is set in launch file

### Camera Calibration Issues

- **Symptom**: Poor alignment between camera data and robot model
- **Solution**:
  - Recalibrate using more sample points
  - Check for physical movement of the camera mount
  - Verify TF tree: `rosrun tf view_frames && evince frames.pdf`

## Advanced Configuration

### Custom Configuration Files

The RealSense ROS package allows loading custom JSON configuration files to set advanced camera parameters:

```bash
roslaunch realsense2_camera rs_camera.launch \
    json_file_path:=$(find kinetirover_perception)/config/d435_settings.json
```

Create a configuration file with the RealSense Viewer and save it to your project.

### Multiple Cameras

If your setup requires multiple RealSense cameras, you can launch them with unique namespaces:

```bash
roslaunch realsense2_camera rs_camera.launch camera:=camera1 serial_no:=<serial_number_1>
roslaunch realsense2_camera rs_camera.launch camera:=camera2 serial_no:=<serial_number_2>
```

You can find the serial numbers using:

```bash
rs-enumerate-devices -s
```

## Next Steps

With the RealSense D435 camera successfully integrated into your KinetiRover project, you're ready to move on to developing perception algorithms. The next tutorial will cover building a perception pipeline for object detection and segmentation.

---

*Previous: [Gazebo Simulation](gazebo-simulation.md) | Next: [Perception Pipeline](perception-pipeline.md)*