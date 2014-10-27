#! /usr/bin/env python

import sys
import rospy
from jarvis_perception import *
import geometry_msgs
from jarvis_perception.srv import *
import jarvis_perception.msg

def return_grips_client():
    rospy.wait_for_service('return_grips')
    try:
        grip_server = rospy.ServiceProxy('return_grips',ReturnGrips)
        grips = grip_server()
        return grips.grips
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "Takes no arguments, returns an array of poses"

if __name__ == "__main__":
    grips = return_grips_client()
    print "Requesting grips"
    print type(grips)
    print type(grips.grasps[1])
    print "grip2 quaternion = " +str(grips.grasps[1].orientation)
