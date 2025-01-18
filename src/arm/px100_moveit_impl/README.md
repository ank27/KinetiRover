# PX100 MoveIt Implementation

## Launch Commands

### Simulation Integration
```bash
# Launch MoveIt with Gazebo
roslaunch px100_moveit_impl px100_moveit_gazebo.launch

# Launch MoveIt with hardware
roslaunch px100_moveit_impl px100_moveit_hardware.launch

# Launch demo implementation
roslaunch px100_moveit_impl demo.launch
```

### Parameters
- `use_rviz`: Launch with RViz (default: true)
- `debug`: Enable debug output (default: false)

[Rest of previous content...]