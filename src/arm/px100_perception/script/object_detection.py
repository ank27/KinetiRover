#!/usr/bin/env python3

import rospy
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3D

class ObjectDetection:
    def __init__(self):
        self.bb_sub = rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3D, self.bb_callback)

    def bb_callback(self, msg):
        print(msg.bounding_boxes.size())


if __name__ == '__main__':
    rospy.init_node('object_detection')
    ObjectDetection()
    rospy.spin()
