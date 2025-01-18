# PX100 Gazebo

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

[Rest of previous content...]