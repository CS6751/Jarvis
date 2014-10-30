#!/usr/bin/env python

from user_interface.srv import *
import rospy

def handle_grip_loc(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return GripLocResponse(req.a + req.b)

def UI_server():
    rospy.init_node('UI_server')
    s = rospy.Service('grip_loc', GripLoc, handle_grip_loc)
    print "Ready to process request."
    rospy.spin()

if __name__ == "__main__":
    UI_server()
