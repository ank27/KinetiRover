/* This script contains moveit code to run the px100 arm using JointPoisitions
*  the idea behind it to execute move group's plan in joint space. 
* usage: use ROS_NAMESPACE=px100 rosrun px100_examples move_interface_node  
*/
#include <px100_moveit_impl/move_group_interface.h>

RobotJointMoveInterface::RobotJointMoveInterface(ros::NodeHandle& nh) {
    // move group intialization for arm
    ROS_INFO("Called constructor ");

    sub_joint_states = nh.subscribe("/px100/joint_states", 1, &RobotJointMoveInterface::joint_state_cb, this);
    ros::Rate loop_rate(loop_hz);
    while (joint_states.position.size() == 0 && ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();

    MoveGroupInterfaceDef::Options options_arm(PLANNING_GROUP_ARM, ROBOT_DESC);
    move_group_interface_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit::planning_interface::MoveGroupInterface(options_arm));
    ROS_INFO("Created move_group_interface_arm ");

    moveit::planning_interface::MoveGroupInterface::Options options_gripper(PLANNING_GROUP_GRIPPER, ROBOT_DESC);
    move_group_interface_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit::planning_interface::MoveGroupInterface(options_gripper));
    ROS_INFO("Created move_group_interface_gripper ");

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    joint_model_group_arm = move_group_interface_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    joint_model_group_gripper = move_group_interface_gripper->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

    move_group_interface_arm->allowReplanning(true);
    move_group_interface_arm->setReplanAttempts(5);
    // get current robot state
    robot_state_ptr = move_group_interface_arm->getCurrentState();

    // set velocity and acceleration limit
    move_group_interface_arm->setMaxVelocityScalingFactor(1.0);
    move_group_interface_arm->setMaxAccelerationScalingFactor(1.0);
    move_group_interface_gripper->setMaxVelocityScalingFactor(1.0);
    move_group_interface_gripper->setMaxAccelerationScalingFactor(1.0);

    // end-effector pose publisher
    ee_pose_pub = nh.advertise<geometry_msgs::Pose>("ee_pose", 10);

    // action client
    execure_trajectory_action_client.reset(
        new actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>(nh, "execute_trajectory", false));
    execure_trajectory_action_client->waitForServer();

    ROS_INFO("Execute Trajectory server is available");

    initRvizVisualization();
    // publish_ee_pose();
};

RobotJointMoveInterface::~RobotJointMoveInterface(){};

void RobotJointMoveInterface::joint_state_cb(const sensor_msgs::JointState &msg) {
    this->joint_states = msg;
}

void RobotJointMoveInterface::printRobotStateInfo() const
{
    std::cout << "---- Printing robot state info -------------" << std::endl;
    this->move_group_interface_arm->getCurrentState()->printStateInfo();
    std::cout << "---- End of robot state info -------------" << std::endl;

};

void RobotJointMoveInterface::setMaxArmVelocityScalingFactor(const double factor)
{
    assert(this->move_group_interface_arm != nullptr);
    this->move_group_interface_arm->setMaxVelocityScalingFactor(factor);
}

void RobotJointMoveInterface::setMaxArmAccelerationScalingFactor(const double factor)
{
    assert(this->move_group_interface_arm != nullptr);
    this->move_group_interface_arm->setMaxAccelerationScalingFactor(factor);
}

void RobotJointMoveInterface::setMaxGripperVelocityScalingFactor(const double factor)
{
    this->move_group_interface_gripper->setMaxVelocityScalingFactor(factor);
}

void RobotJointMoveInterface::setMaxGripperAccelerationScalingFactor(const double factor)
{
    this->move_group_interface_gripper->setMaxAccelerationScalingFactor(factor);
}

moveit::core::RobotStatePtr RobotJointMoveInterface::getCurrentRobotState() const 
{
    return this->move_group_interface_arm->getCurrentState();
};

std::vector<std::string> RobotJointMoveInterface::getNamedTargetsArm() const
{
    return this->move_group_interface_arm->getNamedTargets();
};

std::vector<std::string> RobotJointMoveInterface::getNamedTargetsGripper() const
{
    return this->move_group_interface_gripper->getNamedTargets();
};

void RobotJointMoveInterface::printArmGroupInfo() const
{
    this->joint_model_group_arm->printGroupInfo();
};

void RobotJointMoveInterface::printGripperGroupInfo() const
{
    this->joint_model_group_gripper->printGroupInfo();
};

void RobotJointMoveInterface::printRobotGroups() const
{
    std::vector<std::string> groups = this->move_group_interface_arm->getJointModelGroupNames();
    for(auto& g : groups)
    {
        std::cout << "Robot group present : " << g << std::endl;
    }
    std::cout << " --- End of robot groups ----";
};

void RobotJointMoveInterface::initRvizVisualization()
{
    ROS_INFO("-- initRviz visualization----");
    if(this->enable_rviz_visualization)
    {
        moveit_visual_tools::MoveItVisualTools visual_tools(END_EFFECTOR_LINK);
        this->visual_tools.deleteAllMarkers();
        visual_tools.loadRemoteControl();

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 0.5;
        
        this->visual_tools.publishText(text_pose, "MoveJointSpace Demo", rvt::WHITE, rvt::XLARGE);
        this->visual_tools.trigger();
    }
};

bool RobotJointMoveInterface::get_enable_rviz_visualization()
{
    return this->enable_rviz_visualization;
};

void RobotJointMoveInterface::set_enable_rviz_visualization(bool should_visualize)
{
    this->enable_rviz_visualization = should_visualize;
};

void RobotJointMoveInterface::show_plan_trajectory_rviz(MoveGroupPlan& plan, const moveit::core::JointModelGroup* joint_model_group) {
    this->visual_tools.deleteAllMarkers();
    this->visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    this->visual_tools.trigger();
};

geometry_msgs::Pose RobotJointMoveInterface::getCurrentEndeffectorPose() 
{
    return this->move_group_interface_arm->getCurrentPose().pose;
};

void RobotJointMoveInterface::publish_ee_pose()
{
    while (ros::ok())
    {
        geometry_msgs::Pose current_pose = getCurrentEndeffectorPose();
        this->ee_pose_pub.publish(current_pose);
    }
};

void RobotJointMoveInterface::printCurrentEndeffectorPose() const 
{
    ROS_INFO(" --- Current End-effector Pose -------------");
    geometry_msgs::Point current_pose = this->move_group_interface_arm->getCurrentPose().pose.position;
    geometry_msgs::Quaternion current_orientation = this->move_group_interface_arm->getCurrentPose().pose.orientation;
    ROS_INFO("Current pose X - %f", current_pose.x);
    ROS_INFO("Current pose Y - %f", current_pose.y);
    ROS_INFO("Current pose Z - %f", current_pose.z);
    ROS_INFO("Current orientation x - %f", current_orientation.x);
    ROS_INFO("Current orientation y - %f", current_orientation.y);
    ROS_INFO("Current orientation z - %f", current_orientation.z);
    ROS_INFO("Current orientation w %f", current_orientation.w);
    ROS_INFO("---------------------------------------");
}

MoveGroupPlan RobotJointMoveInterface::plan_arm_joint_position(const std::vector<double> target_joint_positions)
{
    MoveGroupPlan arm_plan;
    this->joint_model_group_arm = this->move_group_interface_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    moveit::core::RobotStatePtr current_state_arm = this->move_group_interface_arm->getCurrentState();

    assert(this->joint_model_group_arm != nullptr);
    std::vector<double> arm_initial_positions;
    current_state_arm->copyJointGroupPositions(this->joint_model_group_arm, arm_initial_positions);
    arm_initial_positions = target_joint_positions;
    this->move_group_interface_arm->setJointValueTarget(arm_initial_positions);
    bool success_arm_plan = (this->move_group_interface_arm->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("tutorial Moving gripper to goal %s", success_arm_plan ? "SUCCESS" : "FAILED");
    return arm_plan;
};

// Execute a given plan using execute method.
bool RobotJointMoveInterface::execute_arm_plan(MoveGroupPlan& plan)
{
    this->move_group_interface_arm->execute(plan);
    this->move_group_interface_arm->getMoveGroupClient().waitForResult(ros::Duration(2.0));
    return true;
};

bool RobotJointMoveInterface::move_arm_to_joint_position(std::vector<double> joints_position)
{
    plan_arm_joint_position(joints_position);
    moveit::core::MoveItErrorCode error_code = this->move_group_interface_arm->move();
    this->move_group_interface_arm->getMoveGroupClient().waitForResult();
    ROS_INFO("Arm Action plan is successfully executed ");
    bool success_traj_execution = (error_code == moveit::core::MoveItErrorCode::SUCCESS);
    return success_traj_execution;
};

MoveGroupPlan RobotJointMoveInterface::plan_gripper(std::vector<double> gripper_position)
{
    MoveGroupPlan gripper_plan;
    this->joint_model_group_gripper = this->move_group_interface_gripper->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
    moveit::core::RobotStatePtr current_state_gripper = this->move_group_interface_gripper->getCurrentState();

    assert(this->joint_model_group_gripper != nullptr);
    std::vector<double> gripper_initial_positions;
    current_state_gripper->copyJointGroupPositions(this->joint_model_group_gripper, gripper_initial_positions);
    gripper_initial_positions = gripper_position;
    this->move_group_interface_gripper->setJointValueTarget(gripper_position);
    bool success_gripper_plan = (this->move_group_interface_gripper->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("tutorial Moving gripper to goal %s", success_gripper_plan ? "SUCCESS" : "FAILED");
    return gripper_plan;
};

bool RobotJointMoveInterface::move_gripper(std::vector<double> gripper_position)
{
    MoveGroupPlan gripper_plan = plan_gripper(gripper_position);
    moveit::core::MoveItErrorCode error_code= this->move_group_interface_gripper->move();
    this->move_group_interface_gripper->getMoveGroupClient().waitForResult();
    ROS_INFO("Gripper Action plan is successfully executed ");
    bool success_traj_execution = (error_code == moveit::core::MoveItErrorCode::SUCCESS);
    return success_traj_execution;
};

bool RobotJointMoveInterface::open_gripper()
{
    return this->move_gripper(this->gripper_open_config);
};

bool RobotJointMoveInterface::close_gripper()
{
    return this->move_gripper(this->gripper_closed_config);
};

bool RobotJointMoveInterface::move_arm_to_home(){
    return this->move_arm_to_joint_position(this->arm_home_config);
}

bool RobotJointMoveInterface::move_arm_to_sleep(){
    return this->move_arm_to_joint_position(this->arm_sleep_config);
}

bool RobotJointMoveInterface::move_arm_to_upright(){
    return this->move_arm_to_joint_position(this->arm_upright_config);
}

bool RobotJointMoveInterface::move_arm_to_joint_positions(std::vector<std::vector<double>> target_positions)
{
    assert(target_positions.size() > 0);
    bool should_execute_next = false;
    std::cout << "TARGET POSITIONS TO MOVE -- " << target_positions.size();
    for(auto& position : target_positions)
    {
        move_arm_to_joint_position(position);
    };
    return true;
};

/**
 * Use cartesian space to generate trajectory and move end-effector to waypoints.
 * 
 * Note: It uses computeCartesianPath which is suitable for straight-line Cartesian path 
 * to a target pose. 
 * However, note that straight-line motion generation is inherently incompatible with 
 * obstacle avoidance.
 * computeCartesianPath does not plan around obstacles 
 * (because that would violate the straight-line motion). 
 * It just stops in front of the obstacle, returning a fraction of the achieved motion 
 * less than 100%.
*/
void RobotJointMoveInterface::set_ee_trajectory(std::vector<geometry_msgs::Pose> waypoints) {
    //delete all markers from rviz
    this->visual_tools.deleteAllMarkers();
    this->joint_model_group_arm = this->move_group_interface_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    moveit::core::RobotStatePtr current_state_arm = this->move_group_interface_arm->getCurrentState();

    assert(this->joint_model_group_arm != nullptr);
    std::vector<geometry_msgs::Pose> ee_waypoints;
    // put initial pose in waypoints.
    geometry_msgs::Pose initial_pose = getCurrentEndeffectorPose();
    ee_waypoints.push_back(initial_pose);
    for (auto& position: waypoints)
    {
        ee_waypoints.push_back(position);  
    }

    // // printing ee_waypoints
    ROS_INFO("-------------");
    for (auto& position: ee_waypoints)
    {
        geometry_msgs::Pose pose = position;
        ROS_INFO_STREAM("EE_WAYPOINTS POSE X:" << pose.position.x << ", Y: " << pose.position.y << ", Z: " << pose.position.z);
        ROS_INFO_STREAM("EE_WAYPOINTS ORIENTATION X:" << pose.orientation.x << ", Y: " << pose.orientation.y << ", Z: " << pose.orientation.z << ", w : " << pose.orientation.w);
        ROS_INFO(" ----------------- ");
    }

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01; //1cm steps
    double fraction = 0.0;
    fraction = this->move_group_interface_arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    this->visual_tools.deleteAllMarkers();
    this->visual_tools.publishTrajectoryLine(trajectory, this->joint_model_group_arm);
    this->visual_tools.trigger();

    MoveGroupPlan arm_plan;
    arm_plan.trajectory_ = trajectory;
    this->move_group_interface_arm->execute(arm_plan);
    // moveit::core::MoveItErrorCode error_code= this->move_group_interface_arm->move();
    this->move_group_interface_arm->getMoveGroupClient().waitForResult();
    ROS_INFO("Cartesian arm move is successfully executed ");
};

/**
 * Move end-effector to a single pose. This is different than set_ee_trajectory, which
 * takes list of pose to move robot in straight line motion.
 * Note: This method is incompatible with obstacle-avoidance.
*/
bool RobotJointMoveInterface::move_ee_pose(geometry_msgs::Pose& pose) {
    MoveGroupPlan arm_plan;
    assert(this->joint_model_group_arm != nullptr);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = this->move_group_interface_arm->getEndEffectorLink();
    pose_stamped.pose = pose;
    // this->set_enable_rviz_visualization(true);

    /**
     * setPoseTarget use IKSolver to find the solutions. which doesn't work as expected some times if the given pose is not correct.
    */
    this->move_group_interface_arm->setPositionTarget(pose.position.x, pose.position.y, pose.position.z);
    
    /**
     * use setPose method, this requires correct position and orientation of end-effector.
    */
    // this->move_group_interface_arm->setPoseTarget(pose);

    /** set joint value target works when giving pose
    */
    // this->move_group_interface_arm->setJointValueTarget(pose);
    bool success = (this->move_group_interface_arm->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    this->show_plan_trajectory_rviz(arm_plan, this->joint_model_group_arm);    

    moveit::core::MoveItErrorCode error_code = this->move_group_interface_arm->move();
    this->move_group_interface_arm->getMoveGroupClient().waitForResult();
    ROS_INFO("Arm end-effector move is successfully executed ");
    bool success_traj_execution = (error_code == moveit::core::MoveItErrorCode::SUCCESS);
    return success;
};