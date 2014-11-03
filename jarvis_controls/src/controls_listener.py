#!/usr/bin/env python

import rospy
from jarvis_planner.msg import planner_trajectories
from jarvis_executive.msg import robot_compliance

def setptcallback(data):
	print 'Robot controls are doing things!'
     #low level API calls here, passing the desired trajectory in.   
def compliancecallback(data):
	print 'Robot compliance is changing!'
     #low level API calls here, passing the desired trajectory in.   

def controls_listener():

    rospy.init_node('controls_listener')

	rospy.Subscriber("planner_trajectories", String, setptcallback)
	rospy.Subscriber("planner_trajectories", String, compliancecallback)

    rospy.spin()
        
if __name__ == '__main__':
    listener()
