#! /usr/bin/env python

import sys
import rospy
from jarvis_perception import *
import geometry_msgs
from jarvis_perception.srv import *
import jarvis_perception.msg
from visualization_msgs.msg import Marker
from std_msgs.msg import String


def grip_loc_client():
    rospy.wait_for_service('return_grips')
    try:
        grip_server = rospy.ServiceProxy('return_grips',ReturnGrips)
        self.grips = grip_server()
        print self.grips
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
