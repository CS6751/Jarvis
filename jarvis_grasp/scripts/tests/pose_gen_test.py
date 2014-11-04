#!/usr/bin/env python

import rospy
from jarvis_perception.msg import GraspArray, GraspBox

def tester():
    rospy.init_node('pose_gen_tester', anonymous=True)
    pub = rospy.Publisher('grasp', GraspArray, queue_size=10)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = GraspArray()
        msg.grasps = [GraspBox(), GraspBox()]
        pub.publish(msg)

if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException: pass
