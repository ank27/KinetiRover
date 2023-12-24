# px100_moveit

## Overview

This package contains the config and launch files to run px100 manipulator using moveit. These files are generated using moveit_setup_assistent, additionally `px100_moveit.launch` file added to run the gazebo, rviz with included moveit setup assistent.

## Note
When running gazebo simulation, make sure to unpause physics engine to publish the transformation data using `rosservice call /gazebo/unapause_physics`
