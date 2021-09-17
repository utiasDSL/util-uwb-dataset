#!/usr/bin/env python3
import roslib
import rospy
from eskf import ESKF_Node

if __name__ == '__main__':
    rospy.init_node("eskf_node", anonymous=True)
    # load params
    
    eskf = ESKF_Node()
    rospy.spin()


