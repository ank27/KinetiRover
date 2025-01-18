# PX100 Perception

## Launch Commands

### RealSense Camera
```bash
# Launch RealSense camera node
roslaunch px100_perception realsense.launch

# Launch perception pipeline
roslaunch px100_perception perception.launch

# Launch with visualization
roslaunch px100_perception perception_rviz.launch
```

### Parameters
- `camera`: Camera name (default: camera)
- `enable_pointcloud`: Enable pointcloud generation (default: true)
- `enable_depth`: Enable depth stream (default: true)

[Rest of previous content...]