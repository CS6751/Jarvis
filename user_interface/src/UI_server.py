#!/usr/bin/env python

import sys
import geometry_msgs
from user_interface.srv import *
import updateWeights
import rospy

def handle_grip_loc(req):
    gripsWithUpdatedWeights = updateWeights.updateWeights(req)
    print "returning updated weights"
    return user_interface.srv.GripLocResponse(gripsWithUpdatedWeights)

def UI_server():
    rospy.init_node('UI_server')
    s = rospy.Service('grip_loc', user_interface.srv.GripLoc, handle_grip_loc)
    print "Ready to process request."
    rospy.spin()

if __name__ == "__main__":
    UI_server()
