# PX100 MoveIt
This package contains the MoveIt configuration for the PX100 robotic manipulator. It was initially generated using the MoveIt Setup Assistant and customized for optimal performance.

## Launch Commands

### Basic MoveIt
```bash
# Launch MoveIt planning pipeline
roslaunch px100_moveit move_group.launch

# Launch with RViz visualization
roslaunch px100_moveit demo.launch

# Launch planning context
roslaunch px100_moveit planning_context.launch
```

### Parameters
- `use_rviz`: Launch with RViz (default: true)
- `pipeline`: Planning pipeline to use (default: ompl)
- `debug`: Launch in debug mode (default: false)
