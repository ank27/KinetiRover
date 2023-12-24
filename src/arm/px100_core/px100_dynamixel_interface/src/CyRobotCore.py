#! /usr/bin/env python3

import yaml
import rospy
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
import numpy as np
import threading
from px100_dynamixel_interface.srv import TorqueEnable, TorqueEnableResponse
from px100_dynamixel_interface.srv import RobotInfo, RobotInfoResponse
from px100_dynamixel_interface.msg import JointTrajectoryCommand

# A robot class responsible to simulate motor data and provide
# current velocity, position of motors.
class CyRobotCore(object):
    def __init__(self):
        self.robot_desc = None
        self.robot_name = None
        self.timer_hz = 20
        self.group_map = {}
        self.motor_map = {}
        self.sleep_map = {}
        self.gripper_map = {}
        self.js_index_map = {}
        self.motor_configs = {}
        self.mode_configs = {}
        self.joint_states = JointState()
        self.joint_state_topic = None
        self.gripper_order = []
        self.joint_state_indices = []
        self.cmd_mutex = threading.Lock()
        self.commands = {}
        self.move_treshold = 300
        self.execute_joint_trajectory = False

        self.get_urdf_info()
        self.robot_get_motor_configs()
        rospy.Service('torque_enable', TorqueEnable, self.robot_srv_torque_enable)
        rospy.Service('get_robot_info', RobotInfo, self.robot_srv_get_robot_info)
        
        rospy.Subscriber('commands/joint_trajectory', JointTrajectoryCommand, self.robot_sub_command_trajectory)
        self.pub_joint_states = rospy.Publisher(self.joint_state_topic, JointState, queue_size=1)
        
        rospy.Timer(rospy.Duration(1.0/self.timer_hz), self.robot_update_joint_states)

    def get_urdf_info(self):
        self.robot_name = rospy.get_namespace().strip("/")
        full_robot_description_name = "/" + self.robot_name + '/robot_description'
        if rospy.has_param(full_robot_description_name):
            self.robot_desc = URDF.from_parameter_server(key= full_robot_description_name)

    def robot_get_motor_configs(self):
        motor_config_filepath = rospy.get_param('~motor_configs')
        mode_config_filepath = rospy.get_param('~mode_configs')

        # try to open both files
        try:
            with open(motor_config_filepath, "r") as yamlfile:
                self.motor_configs = yaml.safe_load(yamlfile)
        except IOError:
            rospy.logerr("Cybot Motor config file was not found")
            return False
        
        try:
            with open(mode_config_filepath, "r") as yamlfile:
                self.mode_configs = yaml.safe_load(yamlfile)
        except IOError:
            rospy.logerr("Cybot Mode config file was not found")
            return False
        
        joint_order = self.motor_configs["joint_order"]
        sleep_positions = self.motor_configs["sleep_positions"]
        self.joint_state_topic = "/px100/" + self.motor_configs["joint_state_publisher"]["topic_name"]
        motor_groups = self.motor_configs.get("groups", {})
        grippers = self.motor_configs.get("grippers", {})
        motors = self.motor_configs["motors"]

        mode_groups = self.mode_configs.get("groups", {})
        singles = self.mode_configs.get("singles", {})

        #populate gripper map
        for gpr, items in grippers.items():
            self.gripper_map[gpr] = {"horn_radius" : items["horn_radius"],
                                     "arm_length" : items["arm_length"],
                                     "left_finger" : items["left_finger"],
                                     "right_finger" : items["right_finger"]}

        # populate self.sleep_map, self.gripper_order, and self.js_index_map
        # also, initialize self.joint_states
        for indx in range(len(joint_order)):
            self.sleep_map[joint_order[indx]] = sleep_positions[indx]
            if joint_order[indx] in self.gripper_map:
                self.gripper_order.append(joint_order[indx])
                self.gripper_map[joint_order[indx]]["js_index"] = indx
            self.js_index_map[joint_order[indx]] = indx
            self.joint_states.name.append(joint_order[indx])
            self.joint_states.position.append(sleep_positions[indx])

        # continue to populate self.joint_states with gripper finger info
        for gpr in self.gripper_order:
            fingers = ["left_finger", "right_finger"]
            lin_dist = self.robot_convert_angular_position_to_linear(gpr, self.sleep_map[gpr])
            for finger in fingers:
                fngr = self.gripper_map[gpr][finger]
                self.js_index_map[fngr] = len(self.js_index_map)
                self.joint_states.name.append(fngr)
                self.joint_states.position.append(lin_dist if finger == "left_finger" else -lin_dist)

        # populate self.motor_map
        for name in joint_order:
            self.motor_map[name] = {"motor_id" : motors[name]["ID"]}

        # populate self.group_map
        groups = list(motor_groups)
        groups.insert(0, "all")
        motor_groups["all"] = joint_order
        mode_groups["all"] = {}
        for grp in groups:
            joint_names = motor_groups[grp]
            joint_group = {}
            joint_group["joint_names"] = joint_names
            joint_group["joint_num"] = len(joint_names)
            joint_group["joint_ids"] = [self.motor_map[name]["motor_id"] for name in joint_names]
            self.group_map[grp] = joint_group
            mode = mode_groups[grp].get("operating_mode", "position")
            profile_type = mode_groups[grp].get("profile_type", "velocity")
            profile_velocity = mode_groups[grp].get("profile_velocity", 0)
            profile_acceleration = mode_groups[grp].get("profile_acceleration", 0)
            self.robot_set_operating_modes("group", grp, mode, profile_type, profile_velocity, profile_acceleration)

        # continue to populate self.motor_map
        for sgl in singles:
            info = singles[sgl]
            mode = info.get("operating_mode", "position")
            profile_type = info.get("profile_type", "velocity")
            profile_velocity = info.get("profile_velocity", 0)
            profile_acceleration = info.get("profile_acceleration", 0)
            self.robot_set_joint_operating_mode(sgl, mode, profile_type, profile_velocity, profile_acceleration)

    def robot_convert_angular_position_to_linear(self, name, angular_position):
        arm_length = self.gripper_map[name]["arm_length"]
        horn_radius = self.gripper_map[name]["horn_radius"]
        a1 = horn_radius * np.math.sin(angular_position)
        c = np.math.sqrt(horn_radius**2 - a1**2)
        a2 = np.math.sqrt(arm_length**2 - c**2)
        return a1 + a2

    def robot_set_joint_operating_mode(self, joint_name, mode, profile_type, profile_velocity, profile_acceleration):
        self.motor_map[joint_name]["mode"] = mode
        self.motor_map[joint_name]["profile_type"] = profile_type
        self.motor_map[joint_name]["profile_velocity"] = profile_velocity
        self.motor_map[joint_name]["profile_acceleration"] = profile_acceleration
        
    def robot_set_operating_modes(self, cmd_type, name, mode, profile_type, profile_velocity, profile_acceleration):
        if(cmd_type == "group" and name in self.group_map):
            for joint in self.group_map[name]["joint_names"]:
                self.robot_set_joint_operating_mode(joint, mode, profile_type, profile_velocity, profile_acceleration)   
            self.group_map[name]["mode"] = mode
            self.group_map[name]["profile_type"] = profile_type
            rospy.loginfo("Cybot operating mode for %s changed to %s." %(name, mode)) 
        elif cmd_type == "single" and name in self.motor_map:
            self.robot_set_joint_operating_mode(name, mode, profile_type, profile_velocity, profile_acceleration)
            rospy.loginfo("Cybot operating mode for %s changed to %s." %(name, mode))
        elif ((cmd_type == "group" and name not in self.group_map) or (cmd_type == "single" and name not in self.motor_map)):
            rospy.loginfo("Cybot The '%s' joint/group does not exist. Was it added to the motor config file?" % name)
        else:
            rospy.logerr("Cybot Invalid command for argument 'cmd_type' while setting operating mode.")

    def robot_srv_torque_enable(self, request):
        if(request.cmd_type == "group"):
            if(request.enable):
                rospy.loginfo("CyBot The group %s has torque on "%request.name)
            else:
                rospy.loginfo("CyBot The group %s has torque off "%request.name)
        else:
            if(request.enable):
                rospy.loginfo("CyBot The group %s has torque on "%request.name)
            else:
                rospy.loginfo("CyBot The group %s has torque off "%request.name)
        return TorqueEnableResponse()

    def robot_srv_get_robot_info(self, request):
        response = RobotInfoResponse()
        joint_name = []
        if(request.cmd_type == "group"):
            joint_names = self.group_map[request.name]["joint_names"]
            response.profile_type = self.group_map[request.name]["profile_type"]
            response.mode = self.group_map[request.name]["mode"]    
        else:
            joint_names.append(request.name)
            response.profile_type = self.motor_map[request.name]["profile_type"]
            response.mode = self.motor_map[request.name]["mode"]
        
        response.num_joints = len(joint_name)

        for name in joint_names:
            response.joint_ids.append(self.motor_map[name]["motor_id"])
            if name in self.gripper_map:
                response.joint_sleep_positions.append(self.robot_convert_angular_position_to_linear(name, 0))
                name = self.gripper_map[name]["left_finger"]
                response.joint_names.append(name)
            else:
                response.joint_sleep_positions.append(self.sleep_map[name])
                response.joint_names.append(name)
            response.joint_state_indices.append(self.js_index_map[name])
            if self.robot_desc is not None:
                joint_object = next((joint for joint in self.robot_desc.joints if joint.name == name), None)
                response.joint_lower_limits.append(joint_object.limit.lower)
                response.joint_upper_limits.append(joint_object.limit.upper)
                response.joint_velocity_limits.append(joint_object.limit.velocity)
        if 'all' not in request.name:
            response.name.append(request.name)
        else:
            for key, _ in self.group_map.items():
                response.name.append(key)
        return response

    def robot_update_joint_states(self, event):
        with self.cmd_mutex:
            for joint in self.commands.copy():
                mode = self.motor_map[joint]["mode"]

                if "position" in mode:
                    if type(self.commands[joint]) == list:
                        value = self.commands[joint].pop()
                        if len(self.commands[joint]) == 0:
                            del self.commands[joint]
                    else:
                        value = self.commands[joint]
                        del self.commands[joint]

                    if joint in self.gripper_map:
                        gpr = self.gripper_map[joint]
                        if mode == "linear_position":
                            angle = self.robot_convert_linear_position_to_radian(joint, value)
                            self.joint_states.position[self.js_index_map[joint]] = angle
                            self.joint_states.position[self.js_index_map[gpr["left_finger"]]] = value / 2.0
                            self.joint_states.position[self.js_index_map[gpr["right_finger"]]] = -value / 2.0
                        else:
                            lin_pos = self.robot_convert_angular_position_to_linear(joint, value)
                            self.joint_states.position[self.js_index_map[joint]] = value
                            self.joint_states.position[self.js_index_map[gpr["left_finger"]]] = lin_pos
                            self.joint_states.position[self.js_index_map[gpr["right_finger"]]] = -lin_pos
                    else:
                        self.joint_states.position[self.js_index_map[joint]] = value

                else:
                    value = self.commands[joint]
                    if value == 0:
                        del self.commands[joint]
                    else:
                        if mode == "velocity":
                            new_val = value / self.timer_hz
                        else:
                            # treat 'pwm' and 'current' equivalently
                            new_val = value / 2000.0
                        self.joint_states.position[self.js_index_map[joint]] += new_val
                        if joint in self.gripper_map:
                            gpr = self.gripper_map[joint]
                            angle = self.joint_states.position[self.js_index_map[joint]]
                            lin_pos = self.robot_convert_angular_position_to_linear(joint, angle)
                            self.joint_states.position[self.js_index_map[gpr["left_finger"]]] = lin_pos
                            self.joint_states.position[self.js_index_map[gpr["right_finger"]]] = -lin_pos

            self.joint_states.header.stamp = rospy.Time.now()
            self.pub_joint_states.publish(self.joint_states)

    def robot_sub_command_trajectory(self, msg):
        if self.execute_joint_trajectory:
            rospy.loginfo("Trajectory is in execution, cancelling this trajectory")
            return
        if len(msg.traj.points) < 2:
            rospy.loginfo("Trajectory has less than 2 points, Aborting...")
            return
        
        mode = None
        joint_name = []
        if msg.cmd_type == "group":
            mode = self.group_map[msg.name]["mode"]
            joint_names = self.group_map[msg.name]["joint_names"]
        elif msg.cmd_type == "single":
            mode = self.motor_map[msg.name]["mode"]
            joint_name.append(msg.name)
        
        if len(msg.traj.points[0].positions) == len(joint_name):
            for x in range(len(joint_name)):
                expected_state = msg.traj.points[0].positions[x]
                actual_state = self.joint_states.position[self.js_index_map[joint_name[x]]]
                if not (abs(expected_state - actual_state) < 0.01):
                    rospy.loginfo(" %s Joint not correct intial state" % joint_name[x])
        
        self.execute_joint_trajectory = True
        points = msg.traj.points
        time_start = rospy.Time.now()
        for x in range(1, len(points)):
            if msg.cmd_type == "group":
                if "position" in mode:
                    self.robot_write_command(msg.name, points[x].positions)
                elif mode == "velocity":
                    self.robot_write_command(msg.name, points[x].velocities)
                elif mode == "pwm" or mode == "current":
                    self.robot_write_command(msg.name, points[x].effort)
            elif msg.cmd_type == "single":
                if "position" in mode:
                    self.robot_write_joint_command(msg.name, points[x].positions[0])
                elif mode == "velocity":
                    self.robot_write_joint_command(msg.name, points[x].velocities[0])
                elif mode == "pwm" or mode == "current":
                    self.robot_write_joint_command(msg.name, points[x].effort[0])
            if x < len(points) - 1:
                period = (points[x].time_from_start - (rospy.Time.now() - time_start))
                rospy.sleep(period)
        
        self.execute_joint_trajectory = False

    def robot_write_command(self, name, command):
        joints = self.group_map[name]["joint_names"]
        for x in range(len(joints)):
            self.robot_write_joint_command(joints[x], command[x])
    
    def robot_write_joint_command(self, name, command):
        motor = self.motor_map[name]
        mode = motor["mode"]
        prof_vel = motor["profile_velocity"]
        with self.cmd_mutex:
            if "position" in mode and prof_vel > self.move_treshold:
                num_itr = int(round(prof_vel / 1000.0 * self.timer_hz + 1))
                self.commands[name] = list(np.linspace(command, self.joint_states.position[self.js_index_map[name]], num_itr))
            else:
                self.commands[name] = command

def main():
    print("Starting ros node")
    rospy.init_node('cybot_core')
    CyRobotCore()
    rospy.spin()

if __name__ == '__main__':
    main()
