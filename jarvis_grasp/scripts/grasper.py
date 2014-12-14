#!/usr/bin/env python

import rospy
from jarvis_grasp.msg import grasp_status
from std_msgs.msg import Bool

def grasper():
    rospy.init_node('grasp_detector', anonymous=True)
    status_pub = rospy.Publisher('grasp_status', grasp_status, queue_size=10)
    grip_pub = rospy.Publisher('gripper', Bool, queue_size=10)
    r = rospy.Rate(10)
    status = False
    while not rospy.is_shutdown():
        key = raw_input()
        status = not status
        msg = Bool()
        msg.data = status
        grip_pub.publish(msg)
        msg = grasp_status()
        msg.is_grasped = status
        status_pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        grasper()
    except rospy.ROSInterruptException: pass
