# PX100 Core

## Launch Commands

### Basic Control
```bash
# Launch core control node
roslaunch px100_core px100_control.launch

# Launch hardware interface
roslaunch px100_core px100_core.launch

# Launch with specific port
roslaunch px100_core px100_control.launch port:=/dev/ttyUSB0
```

### Parameters
- `port`: Dynamixel USB port (default: /dev/ttyUSB0)
- `baud`: Baud rate (default: 1000000)
- `use_fake`: Use fake controllers for testing (default: false)

[Rest of previous content...]