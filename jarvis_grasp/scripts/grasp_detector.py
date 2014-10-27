#!/usr/bin/env python

import rospy
from grasp.msg import grasp_status

def grasp_detector():
    rospy.init_node('grasp_detector', anonymous=True)
    pub = rospy.Publisher('grasp_status', grasp_status, queue_size=10)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = grasp_status()
        msg.is_grasped = True
        pub.publish(msg)

if __name__ == '__main__':
    try:
        grasp_detector()
    except rospy.ROSInterruptException: pass
