# PX100 Description

## Launch Commands

### View Robot Model
```bash
# Launch robot model in RViz
roslaunch px100_description display.launch

# Launch with joint_state_publisher GUI
roslaunch px100_description display.launch gui:=true

# Load robot description to parameter server
roslaunch px100_description upload.launch
```

### Parameters
- `gui`: Launch with joint_state_publisher_gui (default: true)
- `rviz`: Launch with RViz (default: true)

[Rest of previous content...]