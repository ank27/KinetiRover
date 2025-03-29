#!/bin/bash

# Source ROS setup
source /opt/ros/noetic/setup.bash

# Build the workspace if source code is present
if [ -d "/kinetibot_ws/src" ] && [ "$(ls -A /kinetibot_ws/src)" ]; then
    cd /kinetibot_ws
    catkin_make
    source /kinetibot_ws/devel/setup.bash
fi

# Execute the command passed to the container
exec "$@"