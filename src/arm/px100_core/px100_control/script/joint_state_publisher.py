#! /usr/bin/python3

import rospy
from sensor_msgs.msg import JointState

class JointStateManager(object):
    def __init__(self) -> None:
        print("init joint_class")
        self.dynamixel_topic_name = '/dynamixel_workbench/joint_states'
        self.joint_state_topic = '/px100/joint_states'
        
        # subscriber and publisher
        rospy.Subscriber(self.dynamixel_topic_name, JointState, self.sub_dynamixel_joint_state)
        self.publish_joint_states = rospy.Publisher(self.joint_state_topic, JointState, queue_size=10)

    def sub_dynamixel_joint_state(self, joint_state):
        print("got dynamixel joint_state"+str(joint_state))
        joint_states = JointState()
        joint_states.header = joint_state.header

        #manually adding left_finger and right_finger joint_states
        for indx in range(len(joint_state.name)):
            joint_states.name.append(joint_state.name[indx])
            joint_states.position.append(joint_state.position[indx])
            joint_states.velocity.append(joint_state.velocity[indx])
            joint_states.effort.append(joint_state.effort[indx])
        
        joint_states.name.append('left_finger')
        joint_states.position.append(0.0)
        joint_states.velocity.append(0.0)
        joint_states.effort.append(0.0)

        joint_states.name.append('right_finger')
        joint_states.position.append(0.0)
        joint_states.velocity.append(0.0)
        joint_states.effort.append(0.0)

        self.publish_joint_states.publish(joint_states)


if __name__ == "__main__":
    rospy.init_node("joint_state_publisher")
    JointStateManager()
    rospy.spin()