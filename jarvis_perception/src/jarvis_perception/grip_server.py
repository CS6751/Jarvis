#!/usr/bin/env python

import rospy
import jarvis_perception
from jarvis_perception import findGrips
from jarvis_perception.srv import *
import geometry_msgs

class GraspGen:
    'listens to grasp topic and handles service requests for grasps'
    def __init__(self)
        s = rospy.Service('return_grips', jarvis_perception.srv.ReturnGrips, handle_return_grips)
        rospy.spin()
    
    def handle_return_grips(req):
        grips = findGrips.findGrips()
        return jarvis_perception.srv.ReturnGripsResponse(grips)

    

if __name__ == "__main__":
    
    rospy.init_node('return_grips_server')
    g =GraspGen()

