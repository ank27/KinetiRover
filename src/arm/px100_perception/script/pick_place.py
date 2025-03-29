#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class PickAndPlace:
    def __init__(self):
        rospy.init_node('darknet3d_pick_place', anonymous=True)
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Get robot and scene instances
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Setup arm and gripper group commanders
        self.arm_group = moveit_commander.MoveGroupCommander("px100_arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("px100_gripper")
        
        # Set planning parameters
        self.arm_group.set_planning_time(5)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_goal_position_tolerance(0.01)
        self.arm_group.set_goal_orientation_tolerance(0.05)
        
        # Initialize parameters
        self.target_object = rospy.get_param('~target_object', 'bottle')
        self.place_position = rospy.get_param('~place_position', [0.2, 0.2, 0.1])
        
        # Initialize TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Add a delay to allow initialization
        rospy.sleep(2)
        
        # Add a table to the scene
        self.add_table()
        
        # Subscribe to 3D bounding box topic
        self.detection_sub = rospy.Subscriber('/darknet_ros_3d/bounding_boxes', 
                                             BoundingBoxes3d, 
                                             self.detection_callback)
        
        rospy.loginfo("Pick and place node initialized. Waiting for object detections...")
        
    def add_table(self):
        # Add table as a collision object
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0.2
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -0.025  # Half of the table height
        table_pose.pose.orientation.w = 1.0
        
        # Add the box
        self.scene.add_box("table", table_pose, size=(0.5, 0.5, 0.05))
        rospy.sleep(1)  # Wait for the scene to update
        
    def open_gripper(self):
        # Open the gripper to its max position
        gripper_joint_goal = self.gripper_group.get_current_joint_values()
        gripper_joint_goal[0] = 0.037  # Open position for left_finger
        self.gripper_group.go(gripper_joint_goal, wait=True)
        self.gripper_group.stop()
        
    def close_gripper(self):
        # Close the gripper to grip an object
        gripper_joint_goal = self.gripper_group.get_current_joint_values()
        gripper_joint_goal[0] = 0.015  # Closed position for left_finger
        self.gripper_group.go(gripper_joint_goal, wait=True)
        self.gripper_group.stop()
        
    def move_to_home(self):
        # Move the arm to the home position
        joint_goal = self.arm_group.get_current_joint_values()
        joint_goal[0] = 0  # waist
        joint_goal[1] = 0  # shoulder
        joint_goal[2] = 0  # elbow
        joint_goal[3] = 0  # wrist
        
        self.arm_group.go(joint_goal, wait=True)
        self.arm_group.stop()
        
    def detection_callback(self, msg):
        # Skip if no detections
        if len(msg.bounding_boxes) == 0:
            return
        
        # Find our target object
        target_box = None
        for box in msg.boxes:
            if box.class_id == self.target_object:
                target_box = box
                break
        
        if target_box is None:
            return
        
        rospy.loginfo(f"Found target object '{self.target_object}'. Starting pick and place operation.")
        
        # Convert the box pose to gripper pose for picking
        grasp_pose = self.calculate_grasp_pose(target_box)
        
        # Execute pick and place
        self.execute_pick_place(grasp_pose)
        
    def calculate_grasp_pose(self, box):
        # Create a pose for grasping the object
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = box.header.frame_id
        grasp_pose.header.stamp = rospy.Time.now()
        
        # Position slightly above the center of the box
        grasp_pose.pose.position.x = box.center.x
        grasp_pose.pose.position.y = box.center.y
        grasp_pose.pose.position.z = box.center.z + box.size.z/2 + 0.05  # Add offset for approach
        
        # Orientation for top-down grasp
        q = quaternion_from_euler(0, pi/2, 0)  # Roll, Pitch, Yaw
        grasp_pose.pose.orientation.x = q[0]
        grasp_pose.pose.orientation.y = q[1]
        grasp_pose.pose.orientation.z = q[2]
        grasp_pose.pose.orientation.w = q[3]
        
        # Transform to base_link frame if needed
        if grasp_pose.header.frame_id != "base_link":
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    grasp_pose.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
                grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, transform)
                grasp_pose.header.frame_id = "base_link"
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF Error: {e}")
                return None
        
        return grasp_pose
        
    def execute_pick_place(self, grasp_pose):
        if grasp_pose is None:
            rospy.logerr("Invalid grasp pose. Aborting pick and place.")
            return
        
        # Move to home position
        self.move_to_home()
        
        # Open gripper
        self.open_gripper()
        
        # Approach from above
        approach_pose = grasp_pose
        approach_pose.pose.position.z += 0.1  # 10cm above grasp position
        
        # Plan and execute approach
        self.arm_group.set_pose_target(approach_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if not success:
            rospy.logerr("Failed to reach approach position. Aborting pick and place.")
            return
        
        # Plan cartesian path to grasp position
        waypoints = []
        waypoints.append(grasp_pose.pose)  # Move down to grasp
        
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,       # eef_step
            0.0)        # jump_threshold
        
        if fraction < 0.9:
            rospy.logwarn(f"Only achieved {fraction:.2f} of Cartesian path. Aborting.")
            self.move_to_home()
            return
        
        # Execute cartesian path
        self.arm_group.execute(plan, wait=True)
        
        # Close gripper to grasp object
        self.close_gripper()
        rospy.sleep(0.5)  # Wait for gripper to close
        
        # Lift object
        lift_pose = grasp_pose
        lift_pose.pose.position.z += 0.1  # 10cm above grasp position
        
        self.arm_group.set_pose_target(lift_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if not success:
            rospy.logerr("Failed to lift object. Aborting pick and place.")
            return
        
        # Create place pose
        place_pose = PoseStamped()
        place_pose.header.frame_id = "base_link"
        place_pose.pose.position.x = self.place_position[0]
        place_pose.pose.position.y = self.place_position[1]
        place_pose.pose.position.z = self.place_position[2] + 0.1  # Add some height
        place_pose.pose.orientation = lift_pose.pose.orientation  # Keep same orientation
        
        # Move to place position
        self.arm_group.set_pose_target(place_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if not success:
            rospy.logerr("Failed to reach place position. Returning to home.")
            self.move_to_home()
            return
        
        # Lower object to surface
        place_down_pose = place_pose
        place_down_pose.pose.position.z = self.place_position[2]
        
        waypoints = []
        waypoints.append(place_down_pose.pose)
        
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,       # eef_step
            0.0)        # jump_threshold
        
        self.arm_group.execute(plan, wait=True)
        
        # Open gripper to release object
        self.open_gripper()
        rospy.sleep(0.5)  # Wait for gripper to open
        
        # Move away
        retreat_pose = place_down_pose
        retreat_pose.pose.position.z += 0.1  # Move up 10cm
        
        self.arm_group.set_pose_target(retreat_pose)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        # Return to home position
        self.move_to_home()
        
        rospy.loginfo("Pick and place operation completed successfully!")

if __name__ == '__main__':
    try:
        pick_place = PickAndPlace()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

