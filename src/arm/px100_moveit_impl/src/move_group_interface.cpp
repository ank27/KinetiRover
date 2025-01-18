/* This script contains moveit code to run the px100 arm using JointPoisitions
*  the idea behind it to execute move group's plan in joint space. 
* usage: use ROS_NAMESPACE=px100 rosrun px100_examples move_interface_node  
*/
#include <px100_moveit_impl/move_group_interface.h>


int main(int argc, char** argv)
{
    std::cout << "- --- in main ----- " << std::endl;
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle("/px100");

    RobotJointMoveInterface robotJointMoveInterface(node_handle);
    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Starting move group example");
    // robotJointMoveInterface.set_enable_rviz_visualization(true);
    robotJointMoveInterface.printRobotStateInfo();
    robotJointMoveInterface.printRobotGroups();
    robotJointMoveInterface.printArmGroupInfo();
    robotJointMoveInterface.printGripperGroupInfo();
    const std::vector<std::string> arm_targets = robotJointMoveInterface.getNamedTargetsArm();
    for(const auto t : arm_targets) {
        ROS_INFO("Named arm targets are %s", t.c_str());
    };

    const std::vector<std::string> gripper_targets = robotJointMoveInterface.getNamedTargetsGripper();
    for(const auto t : arm_targets){
        ROS_INFO("Named gripper targets are %s", t.c_str());
    };
    
    ROS_INFO(" ---------- Getting Current pose ---------------");
    robotJointMoveInterface.printCurrentEndeffectorPose();

    /**
     * Move end-effector to cartesian space, this will move the robot to straight line motion
     * which doesn't check for collision avoidance. Also, it does not have more control over
     * the movement of the arm.
    */
    bool move_cartsian_space = false;
    if(move_cartsian_space) {
        std::vector<geometry_msgs::Pose> waypoints;
        // geometry_msgs::Pose pose(robotJointMoveInterface.getCurrentEndeffectorPose());
        // waypoints.push_back(pose);

        geometry_msgs::Pose target_pose3;

        // move to home
        target_pose3.position.x = 0.24;
        target_pose3.position.y = 0.0;
        target_pose3.position.z = 0.20;
        target_pose3.orientation.w = 1.0;
        waypoints.push_back(target_pose3);

        target_pose3.position.x = 0.0;
        target_pose3.position.y = -0.20;
        target_pose3.position.z = 0.10;
        //calculate quaternion from eular angles for end-effector frame.
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, -M_PI/2);
        quat.normalize();
        target_pose3.orientation.x = quat[0];
        target_pose3.orientation.y = quat[1];
        target_pose3.orientation.z = quat[2];
        target_pose3.orientation.w = quat[3];

        waypoints.push_back(target_pose3);

        // move a bit lower to collect object
        target_pose3.position.z -=0.05;
        waypoints.push_back(target_pose3);

        // move to home
        target_pose3.position.x = 0.24;
        target_pose3.position.y = 0.0;
        target_pose3.position.z = 0.20;
        target_pose3.orientation.w = 1.0;
        waypoints.push_back(target_pose3);

        // move to right
        target_pose3.position.x = 0.0;
        target_pose3.position.y = 0.20;
        target_pose3.position.z = 0.10;

        quat.setRPY(0.0, 0.0, -M_PI/2);
        quat.normalize();
        target_pose3.orientation.x = quat[0];
        target_pose3.orientation.y = quat[1];
        target_pose3.orientation.z = quat[2];
        target_pose3.orientation.w = quat[3];

        waypoints.push_back(target_pose3); //move right

        // move lower to release object
        target_pose3.position.z -=0.05;
        waypoints.push_back(target_pose3);

        robotJointMoveInterface.set_ee_trajectory(waypoints);
    }


    /**
     * Move end-effector to cartesian space using individual poses.
     * This doesn't check for collision avoidance. but it does allow to have more control over
     * the movement of the arm, for eg. move -> pick -> move -> place.
    */
    bool move_individual_ee_pose = true;
    if(move_individual_ee_pose) {
        geometry_msgs::Pose ee_pose;
        ee_pose.position.x = 0.0;
        ee_pose.position.y = -0.20;
        ee_pose.position.z = 0.13;

        /**
         * In order to provide correct frame orientation, we could convert 
         * final eular angle of the end-effector frame to quaternion and 
         * pass it to geometry_msgs.Pose.
        */
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, -M_PI/2);
        quat.normalize();        
        ee_pose.orientation.x = quat[0];
        ee_pose.orientation.y = quat[1];
        ee_pose.orientation.z = quat[2];
        ee_pose.orientation.w = quat[3];

        robotJointMoveInterface.move_ee_pose(ee_pose);
        robotJointMoveInterface.open_gripper();
        
        // move a bit low        
        ee_pose.position.z -= 0.04;
        robotJointMoveInterface.move_ee_pose(ee_pose);

        // pick the object
        robotJointMoveInterface.close_gripper();

        robotJointMoveInterface.move_arm_to_joint_position(robotJointMoveInterface.arm_home_config);
        
        // geometry_msgs::Pose current_ee_pose = robotJointMoveInterface.getCurrentEndeffectorPose();
        // current_ee_pose.position.x = 0.0;
        // current_ee_pose.position.y = 0.20;
        // current_ee_pose.position.z = 0.13;
        // quat.setRPY(0.0, 0.0, M_PI/2);
        // quat.normalize();        
        // current_ee_pose.orientation.x = quat[0];
        // current_ee_pose.orientation.y = quat[1];
        // current_ee_pose.orientation.z = quat[2];
        // current_ee_pose.orientation.w = quat[3];

        // robotJointMoveInterface.move_ee_pose(current_ee_pose);

        // open gripper to release the object
        robotJointMoveInterface.open_gripper();

        // go to sleep position
        robotJointMoveInterface.move_arm_to_sleep();
    }
    
    /**
     * move arm to multiple position in joint space.
     * every input contains desired joint positions.
     * since this is moved in joint space, it doesn't require to calculations of IK, 
     * just basic FK is good enough and moving to all positions are most guaranteed to success.
    */
    bool move_to_multiple_position = false;
    if(move_to_multiple_position) {
        // move arm to multiple positions
        std::vector<std::vector<double>> multiple_position_targets;
        std::vector<double> joint_pose;

        // upright
        multiple_position_targets.push_back(robotJointMoveInterface.arm_upright_config);

        // move wrist back and forth
        double wrist_angle = -0.87;
        double elbow_angle = -0.87;
        for (size_t i = 0; i < 5; i++)
        {
            joint_pose.push_back(0.0);
            joint_pose.push_back(0.0);
            joint_pose.push_back(elbow_angle);
            joint_pose.push_back(wrist_angle);
            multiple_position_targets.push_back(joint_pose);
            
            wrist_angle = -wrist_angle;
            if(i%2==0){
                elbow_angle = -1.50;
            } else {
                elbow_angle = -0.87;
            }
            joint_pose.clear();
        }
        joint_pose.clear();
        // Pose right
        joint_pose.push_back(M_PI/2);
        joint_pose.push_back(M_PI/6);
        joint_pose.push_back(0.0);
        joint_pose.push_back(-M_PI/6);
        multiple_position_targets.push_back(joint_pose);
        multiple_position_targets.push_back(robotJointMoveInterface.arm_sleep_config);

        // execute multiple joint position targets
        robotJointMoveInterface.move_arm_to_joint_positions(multiple_position_targets);
    }


    // Add box to pick
    bool use_pick_place_config = false;
    if(use_pick_place_config) {
        int32_t no_of_objects = 1;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(no_of_objects);

        for (size_t i = 0; i < no_of_objects; i++)
        {
            ROS_INFO_STREAM("ADDING COLLISION OBJECT " << i);
            moveit_msgs::CollisionObject object;
            object.id = "box1";
            object.header.frame_id = "px100/base_link";

            // define shape of the object to world
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.02;
            primitive.dimensions[1] = 0.02;
            primitive.dimensions[2] = 0.08;

            /* A pose for the box (specified relative to frame_id) */
            geometry_msgs::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = 0.0;
            box_pose.position.y = -0.2;
            box_pose.position.z = 0.02;

            object.primitives.push_back(primitive);
            object.primitive_poses.push_back(box_pose);
            object.operation = object.ADD;

            collision_objects.push_back(object);
        }

        // //apply collision objects to planning_scene
        robotJointMoveInterface.planning_scene_interface.applyCollisionObjects(collision_objects);

        // addCollisionObjects(robotJointMoveInterface.planning_scene_interface);
        //once we added an object, we plan the robot to move just above the object.
        geometry_msgs::Pose ee_pose_pick;
        ee_pose_pick.position.x = 0.0;
        ee_pose_pick.position.y = -0.20;
        ee_pose_pick.position.z = 0.12;

        /**
         * In order to provide correct frame orientation, we could convert 
         * final eular angle of the end-effector frame to quaternion and 
         * pass it to geometry_msgs.Pose.
        */
        tf2::Quaternion pick_quat;
        pick_quat.setRPY(0.0, 0.0, -M_PI/2);
        pick_quat.normalize();        
        ee_pose_pick.orientation.x = pick_quat[0];
        ee_pose_pick.orientation.y = pick_quat[1];
        ee_pose_pick.orientation.z = pick_quat[2];
        ee_pose_pick.orientation.w = pick_quat[3];

        robotJointMoveInterface.move_ee_pose(ee_pose_pick);
        robotJointMoveInterface.open_gripper();
        
        // move a bit low        
        ee_pose_pick.position.z -= 0.04;
        robotJointMoveInterface.move_ee_pose(ee_pose_pick);

        // pick the object
        robotJointMoveInterface.close_gripper();
        
        robotJointMoveInterface.move_arm_to_joint_position(robotJointMoveInterface.arm_home_config);
    }
    
    while(!ros::ok()) {
       ros::shutdown(); 
    }

    return 0;
}
