#!/usr/bin/env python

import sys
import rospy
from user_interface.srv import *

def grip_loc_client(x, y):
    rospy.wait_for_service('grip_loc')
    try:
        grip_loc = rospy.ServiceProxy('grip_loc', GripLoc)
        resp1 = grip_loc(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, grip_loc_client(x, y))
