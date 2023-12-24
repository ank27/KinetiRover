#! /usr/bin/env python3 

import rospy
import threading
import sys
from px100_dynamixel_interface.srv import TorqueEnable
from px100_dynamixel_interface.srv import RobotInfo
from sensor_msgs.msg import JointState

class CyArmBase(object):
    def __init__(self, robot_model, group_name="arm",gripper_name="gripper", robot_name=None, 
                 moving_time=2.0, accel_time=0.3, gripper_pressure=0.5, 
                 gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350, 
                 init_node=True):
        self.core = CyCore(robot_model, robot_name, init_node)
        self.arm = CyArm(robot_model, robot_name, init_node)

class CyCore(object):
    def __init__(self, robot_model, init_node = True, joint_state_topic="joint_state") -> None:
        self.joint_state = None
        self.js_mutex = threading.Lock()
        self.robot_model = robot_model

        if(self.robot_model is None):
            self.robot_model = robot_model
        if(init_node):
            rospy.init_node(self.robot_model + "robot_manipulation")

        try:
            rospy.wait_for_service("/" + self.robot_model + "/get_robot_info")
            rospy.wait_for_service("/" + self.robot_model + "/torque_enable")
        except rospy.exceptions.ROSException as e:
            print(str(e.args[0]))
            print((
                "The robot '%s' is not discoverable. "
                "Did you enter the correct robot_name parameter? "
                "Is the CyRobotCore node running? "
                "Quitting..." % self.robot_model))
            sys.exit(1)        

        # listen to service data.
        self.srv_robot_info = rospy.ServiceProxy("/"+self.robot_model + "/get_robot_info",RobotInfo)
        self.srv_get_torque = rospy.ServiceProxy("/"+self.robot_model + "/torque_enable", TorqueEnable)
        # subscribe joint_state topic info.
        self.subscribe_joint_states = rospy.Subscriber("/" + self.robot_model + joint_state_topic, JointState, self.joint_state_cb)
        while(self.joint_state == None and not rospy.is_shutdown()): pass
        self.js_index_map = dict(zip(self.joint_state.name, range(len(self.joint_state.name))))
        rospy.sleep(0.5)
        print("Robot : %s, RobotName %s"%(self.robot_model, self.robot_model))
        print("Core initialize successfully")

    def get_robot_info(self, cmd_type, name):
        response = self.srv_robot_info(cmd_type, name)
        return response
    
    def get_torque_info(self, cmd_type, name, enable):
        response = self.srv_get_torque(cmd_type, name, enable)

    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_state = msg


class CyArm(object):
    def __init__(self, core, robot_model, robot_name, init_node) -> None:
        return
