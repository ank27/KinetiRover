# px100_moveit

## Overview

This package contains the necessary config files for px100 arm working with MoveIt. The MoveIt Setup Assistant wizard is used to generate this package. 

This package makes use of the FollowJointTrajectory interface which seems to work pretty well in both Gazebo and on the physical robot. A 'master' launch file [px100_moveit] was written to allow a user to choose whether to have MoveIt work with the simulated version, the physical robot hardware, or a MoveIt generated fake robot.
