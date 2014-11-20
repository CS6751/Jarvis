#!/usr/bin/env python

import rospy
import jarvis_perception
from jarvis_perception import findGrips
from jarvis_perception.srv import *
from jarvis_perception.msg import AxisAlignedBox
import geometry_msgs


def handle_return_grips(req):
    grips = findGrips.findGrips()
    return jarvis_perception.srv.ReturnGripsResponse(grips)

def return_grips_server():
    rospy.init_node('return_grips_server')
    s = rospy.Service('return_grips', jarvis_perception.srv.ReturnGrips, handle_return_grips)
    print 'ready to return grips'

    rospy.spin()

if __name__ == "__main__":
    return_grips_server()
