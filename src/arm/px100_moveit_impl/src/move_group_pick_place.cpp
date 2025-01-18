/* This script contains moveit code to run the px100 arm using JointPoisitions
*  the idea behind it to execute move group's plan in joint space. 
* usage: use ROS_NAMESPACE=px100 rosrun px100_examples move_interface_node  
*/

#include <px100_moveit_impl/move_group_interface.h>

int main(int argc, char** argv)
{
    std::cout << "- --- in main ----- " << std::endl;
    ros::init(argc, argv, "move_group_pick_place");
    ros::NodeHandle node_handle("/px100");

    // RobotJointMoveInterface robotJointMoveInterface(node_handle);
    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    // ros::AsyncSpinner spinner(1);
    // spinner.start();

    ROS_INFO("Starting move group example");
    // robotJointMoveInterface.set_enable_rviz_visualization(true);
    // robotJointMoveInterface.printRobotGroups();
    // robotJointMoveInterface.printArmGroupInfo();
    // robotJointMoveInterface.printGripperGroupInfo();

    // ROS_INFO(" ---------- Getting Current End effector pose ---------------");
    // robotJointMoveInterface.printCurrentEndeffectorPose();

    // // add an object to pick
    // int32_t no_of_objects = 1;
    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.resize(no_of_objects);

    // for (size_t i = 0; i < no_of_objects; i++)
    // {
    //     ROS_INFO_STREAM("ADDING COLLISION OBJECT " << i);
    //     moveit_msgs::CollisionObject object;
    //     object.id = "box1";
    //     object.header.frame_id = "px100/base_link";

    //     // define shape of the object to world
    //     shape_msgs::SolidPrimitive primitive;
    //     primitive.type = primitive.BOX;
    //     primitive.dimensions.resize(3);
    //     primitive.dimensions[0] = 0.02;
    //     primitive.dimensions[1] = 0.02;
    //     primitive.dimensions[2] = 0.04;

    //     /* A pose for the box (specified relative to frame_id) */
    //     geometry_msgs::Pose box_pose;
    //     box_pose.orientation.w = 1.0;
    //     box_pose.position.x = 0.0;
    //     box_pose.position.y = -0.2;
    //     box_pose.position.z = 0.0;

    //     object.primitives.push_back(primitive);
    //     object.primitive_poses.push_back(box_pose);
    //     object.operation = object.ADD;

    //     collision_objects.push_back(object);
    // }

    while(!ros::ok()) {
       ros::shutdown(); 
    }
    // ros::shutdown();
    return 0;
}
