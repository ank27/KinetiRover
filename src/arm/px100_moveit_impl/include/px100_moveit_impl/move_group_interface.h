/* This script contains moveit code to run the px100 arm using JointPoisitions
*  the idea behind it to execute move group's plan in joint space. 
* usage: use ROS_NAMESPACE=px100 rosrun px100_moveit_impl move_group_interface  
*/
#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <typeinfo>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <actionlib/client/simple_action_client.h>

namespace rvt = rviz_visual_tools;
typedef moveit::planning_interface::MoveGroupInterface::Plan MoveGroupPlan;
typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterfaceDef;

class RobotJointMoveInterface {
private:
    const std::string PLANNING_GROUP_ARM = "px100_arm";
    const std::string PLANNING_GROUP_GRIPPER = "px100_gripper";
    const std::string ROBOT_DESC = "robot_description";
    const std::string END_EFFECTOR_LINK = "px100/ee_gripper_link";
    ros::NodeHandle nh;
    bool enable_rviz_visualization = true; // for testing, enable rviz planning visualization.

    // rviz visualization tooling
    moveit_visual_tools::MoveItVisualTools visual_tools;
    sensor_msgs::JointState joint_states;
    const double loop_hz = 10;
    ros::Subscriber sub_joint_states;
    ros::Subscriber move_action_subscriber;
    ros::Publisher ee_pose_pub;
public:
    const std::vector<double> gripper_closed_config{0.015, -0.015}; //[left_finger_position, right_finger_position]
    const std::vector<double> gripper_open_config{0.037, -0.037};
    const std::vector<double> arm_home_config{0.0,0.0,0.0,0.0};     //waist, shoulder, elbow, wrist_angle;
    const std::vector<double> arm_sleep_config{0.0,-1.88,1.5,0.8};  //waist, shoulder, elbow, wrist_angle;
    const std::vector<double> arm_upright_config{0.0,0.0,-M_PI/2,0.0};     //waist, shoulder, elbow, wrist_angle;
    
    moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_arm;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_gripper;
    moveit::core::RobotStatePtr robot_state_ptr;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw model group pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group_arm;
    const moveit::core::JointModelGroup* joint_model_group_gripper;
    std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>> execure_trajectory_action_client;

    RobotJointMoveInterface(ros::NodeHandle& nh);

    ~RobotJointMoveInterface();

    void joint_state_cb(const sensor_msgs::JointState &msg);

    void printRobotStateInfo() const;

    void setMaxArmVelocityScalingFactor(const double factor);
    void setMaxArmAccelerationScalingFactor(const double factor);
    void setMaxGripperVelocityScalingFactor(const double factor);

    void setMaxGripperAccelerationScalingFactor(const double factor);
    
    moveit::core::RobotStatePtr getCurrentRobotState() const;
    std::vector<std::string> getNamedTargetsArm() const;    
    std::vector<std::string> getNamedTargetsGripper() const;
    void printArmGroupInfo() const;
    void printGripperGroupInfo() const;

    void printRobotGroups() const;

    void initRvizVisualization();

    bool get_enable_rviz_visualization();

    void set_enable_rviz_visualization(bool should_visualize);

    void show_plan_trajectory_rviz(MoveGroupPlan& plan, const moveit::core::JointModelGroup* joint_model_group);

    geometry_msgs::Pose getCurrentEndeffectorPose();
    void printCurrentEndeffectorPose() const;

    MoveGroupPlan plan_arm_joint_position(const std::vector<double> target_joint_positions);

    // Execute a given plan using execute method.
    bool execute_arm_plan(MoveGroupPlan& plan);

    bool move_arm_to_joint_position(std::vector<double> joints_position);

    MoveGroupPlan plan_gripper(std::vector<double> gripper_position);

    bool move_gripper(std::vector<double> gripper_position);

    bool open_gripper();
    
    bool close_gripper();

    bool move_arm_to_home();

    bool move_arm_to_sleep();

    bool move_arm_to_upright();

    bool move_arm_to_joint_positions(std::vector<std::vector<double>> target_positions);

    /**
     * Use cartesian space to generate trajectory and move end-effector to waypoints
    */
    void set_ee_trajectory(std::vector<geometry_msgs::Pose> waypoints);

    /**
     * Move end-effector to specific pose
    */
    bool move_ee_pose(geometry_msgs::Pose& pose);

    void publish_ee_pose();

};
