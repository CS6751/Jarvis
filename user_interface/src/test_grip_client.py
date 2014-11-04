#!/usr/bin/env python

import sys
import rospy
import geometry_msgs
from user_interface.srv import *
from user_interface import *

def grip_loc_client():
    rospy.wait_for_service('grip_loc')
    try:
        grip_loc = rospy.ServiceProxy('grip_loc', GripLoc)
        response = grip_loc()
        return response.grips
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "requests grip positions/weights from the perception module."

if __name__ == "__main__":
    print len(sys.argv)
    if len(sys.argv) > 1:
        print usage()
        sys.exit(1)
    else:
        gripLoc = grip_loc_client()
    print "(%s)"%(gripLoc)
