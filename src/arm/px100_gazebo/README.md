# PX100 Gazebo
This package contains the necessary configuration and launch files for simulating the PX100 robotic manipulator in the Gazebo physics simulator.

## Launch Commands

### Simulation
```bash
# Launch basic Gazebo simulation
roslaunch px100_gazebo px100_gazebo.launch

# Launch with custom world
roslaunch px100_gazebo px100_gazebo.launch world:=custom_world.world

# Launch Gazebo controllers
roslaunch px100_gazebo px100_control.launch
```

### Parameters
- `paused`: Start Gazebo paused (default: false)
- `use_sim_time`: Use simulation time (default: true)
- `gui`: Launch Gazebo GUI (default: true)

