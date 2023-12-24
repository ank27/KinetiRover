# px100_gazebo

## Overview

This package contains the necessary config files to get px100 arm simulated in Gazebo. Specifically, it contains the [px100_texture.gazebo](config/px100_texture.gazebo) file which allows the black texture of the robotic arms to display properly. It also contains YAML files with tuned PID gains for the arm and gripper joints so that ros_control can control the arms effectively. This package has one of two applications. It can either be used in conjunction with MoveIt via the FollowJointTrajectory interface or it can be used by itself via the JointPositionController interface.

## Note
When running gazebo simulation, make sure to unpause physics engine to publish the transformation data using `rosservice call /gazebo/unapause_physics`
