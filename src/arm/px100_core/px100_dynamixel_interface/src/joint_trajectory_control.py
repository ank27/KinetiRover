#! /usr/bin/env python3

from px100_dynamixel_interface.msg import JointTrajectoryCommand
from trajectory_msgs import *
import rospy
from time import sleep
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

# This script commands an arbitrary trajectory to the arm joints
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s'
# Then change to this directory and type 'python joint_trajectory_control  # python3 bartender.py if using ROS Noetic'

def main():
    pub = rospy.Publisher('commands/joint_trajectory', JointTrajectoryCommand, queue_size=1)
    rospy.init_node('joint_trajectory_publisher', anonymous = True)

    jointTrajectoryCommand = JointTrajectoryCommand()
    jointTrajectory = JointTrajectory()
    while not rospy.is_shutdown():
        trajectoryPoint1 = JointTrajectoryPoint()
        trajectoryPoint1.positions = [0.0,  1.57, 0.0, 0.0, 0.2]
        jointTrajectory.points.append(trajectoryPoint1)
        
        trajectoryPoint2 = JointTrajectoryPoint()
        trajectoryPoint2.positions = [0.0,  1.57, 1.57, 0.0, -0.2]
        jointTrajectory.points.append(trajectoryPoint2)
        
        trajectoryPoint3 = JointTrajectoryPoint()
        trajectoryPoint3.positions = [1.57,  1.57, 0.0, 0.0, -0.2]
        jointTrajectory.points.append(trajectoryPoint3)

        jointTrajectoryCommand.cmd_type = "group"
        jointTrajectoryCommand.name = "arm"
        jointTrajectoryCommand.traj = jointTrajectory
        pub.publish(jointTrajectoryCommand)
        sleep(6.0)  # sleep to ensure trajectory has time to complete

if __name__ == '__main__':
    main()
