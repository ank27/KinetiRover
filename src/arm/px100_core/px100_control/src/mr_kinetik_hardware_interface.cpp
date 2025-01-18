#include <ros/ros.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include "px100_dynamixel_interface/JointGroupCommand.h"
#include "px100_dynamixel_interface/JointSingleCommand.h"
#include "px100_dynamixel_interface/RobotInfo.h"
#include <sensor_msgs/JointState.h>

class MrKinetikHardwareInterface: public hardware_interface::RobotHW
{
public:
  // hardware interfaces to registera
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface position_joint_interface;
  joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface;

  std::vector<std::string> joint_names{"waist", "shoulder", "elbow", "wrist_angle", "left_finger"};
  std::vector<double> joint_positions{0.0,  0.0,  0.0,  0.0,  0.0};
  std::vector<double> joint_velocities{0.0,  0.0,  0.0,  0.0,  0.0};
  std::vector<double> joint_efforts{0.0,  0.0,  0.0,  0.0,  0.0};
  std::vector<double> joint_position_commands{0.0,  0.0,  0.0,  0.0,  0.0};
  std::vector<float> joint_commands_prev{0.0,  0.0,  0.0,  0.0,  0.0};
  std::vector<int16_t> joint_state_indices{0,1,2,3,5};

  ros::NodeHandle nh;
  ros::Publisher pub_group;
  ros::Publisher pub_gripper;
  ros::Subscriber sub_joint_states;
  ros::ServiceClient srv_robot_info;
  ros::Timer tmr_control_loop;
  ros::Duration elapsed_time;
  float gripper_cmd_prev;
  std::string group_name;
  std::string gripper_name;
  ros::Time curr_update_time, prev_update_time;

  double loop_hz;
  size_t num_joints = 5;
  sensor_msgs::JointState joint_states;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager;

  MrKinetikHardwareInterface(ros::NodeHandle& nh) : nh(nh)
  {
    std::string js_topic;
    nh.getParam("hardware_interface/loop_hz", loop_hz);
    nh.getParam("hardware_interface/group_name", group_name);
    nh.getParam("hardware_interface/gripper_name", gripper_name);
    nh.getParam("hardware_interface/joint_states_topic", js_topic);
    pub_group = nh.advertise<px100_dynamixel_interface::JointGroupCommand>("commands/joint_group", 1);
    pub_gripper = nh.advertise<px100_dynamixel_interface::JointSingleCommand>("commands/joint_single", 1);
    sub_joint_states = nh.subscribe(js_topic, 1, &MrKinetikHardwareInterface::joint_state_cb, this);
    srv_robot_info = nh.serviceClient<px100_dynamixel_interface::RobotInfo>("get_robot_info");

    ros::Rate loop_rate(loop_hz);
    while (joint_states.position.size() == 0 && ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

    // Initialize the joint_position_commands vector to the current joint states
    for (size_t i{0}; i < num_joints; i++)
    {
      std::cout << " group_info_Response joint_indices at : " << i << " is : " << joint_state_indices.at(i) << std::endl;
      std::cout << " group_info_Response joint_position at : " << i << " is : " << joint_position_commands.at(i) << std::endl;
      std::cout << " group_info_Response joint_names at : " << i << " is : " << joint_names.at(i) << std::endl;

      joint_position_commands.at(i) = joint_states.position.at(joint_state_indices.at(i));
      joint_commands_prev.at(i) = joint_position_commands.at(i);
    }
    joint_commands_prev.resize(num_joints - 1);
    gripper_cmd_prev = joint_states.position.at(joint_state_indices.back()) * 2;

    urdf::Model model;
    std::string robot_name = nh.getNamespace();
    urdf::JointConstSharedPtr ptr;
    model.initParam(robot_name + "/robot_description");

    // Initialize Controller
    for (int i = 0; i < num_joints; ++i) {
      // Create joint state interface
      hardware_interface::JointStateHandle jointStateHandle(
        joint_names.at(i),
        &joint_positions.at(i),
        &joint_velocities.at(i),
        &joint_efforts.at(i)
      );
      joint_state_interface.registerHandle(jointStateHandle);

      joint_limits_interface::JointLimits limits;
      ptr = model.getJoint(joint_names.at(i));
      getJointLimits(ptr, limits);
      getJointLimits(joint_names.at(i), nh, limits);

      // Create position joint interface
      hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_commands.at(i));
      joint_limits_interface::PositionJointSaturationHandle jointPositionSaturationHandle(jointPositionHandle, limits);
      position_joint_saturation_interface.registerHandle(jointPositionSaturationHandle);
      position_joint_interface.registerHandle(jointPositionHandle);
    }

    registerInterface(&joint_state_interface);
    registerInterface(&position_joint_interface);
    registerInterface(&position_joint_saturation_interface);


    // once we register all the interfaces, create controller manager and 
    // start control loop timer.
    controller_manager.reset(new controller_manager::ControllerManager(this, nh));
    ros::Duration update_freq = ros::Duration(1.0/loop_hz);
    tmr_control_loop = nh.createTimer(
      update_freq, 
      &MrKinetikHardwareInterface::control_loop, 
      this
    );

  }

  /** Control loop that will call on every tick and update controller manager
   * it will first call read() which reads joint_states, then update controller manager lifecycle
   * and call write() method to publish the joints/sensor data to specific topics.
  */
  void control_loop(const ros::TimerEvent& e)
  {
      elapsed_time = ros::Duration(e.current_real - e.last_real);
      read();
      controller_manager->update(ros::Time::now(), elapsed_time);
      write(elapsed_time);
  }

  void read()
  {
    for (int i = 0; i < num_joints; i++)
    {
      joint_positions.at(i) = joint_states.position.at(joint_state_indices.at(i));
      joint_velocities.at(i) = joint_states.velocity.at(joint_state_indices.at(i));
      joint_efforts.at(i) = joint_states.effort.at(joint_state_indices.at(i));
    }
  }

  void write(ros::Duration elapsed_time)
  {
    px100_dynamixel_interface::JointGroupCommand group_msg;
    px100_dynamixel_interface::JointSingleCommand gripper_msg;
    group_msg.name = group_name;
    gripper_msg.name = gripper_name;
    gripper_msg.cmd = joint_position_commands.back() * 2;

    position_joint_saturation_interface.enforceLimits(elapsed_time);

    for (size_t i{0}; i < num_joints - 1; i++)
      group_msg.cmd.push_back(joint_position_commands.at(i));

    if (joint_commands_prev != group_msg.cmd)
    {
      pub_group.publish(group_msg);
      joint_commands_prev = group_msg.cmd;
    }
    if (gripper_cmd_prev != gripper_msg.cmd)
    {
      pub_gripper.publish(gripper_msg);
      gripper_cmd_prev = gripper_msg.cmd;
    }
  }

  void joint_state_cb(const sensor_msgs::JointState &msg)
  {
    joint_states = msg;
  }

  ~MrKinetikHardwareInterface(){};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mr_kinetik_hardware_interface");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    MrKinetikHardwareInterface MrKinetikHardwareInterface(nh);
    spinner.spin();
    return 0;
}
