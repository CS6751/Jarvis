#!usr/bin/env python

import rospy
from std_msgs.msg import String

def trajectory_publisher(name):
    pub = rospy.Publisher('trajectory', String, queue_size=10)
    rospy.init_node('trajectory')
    pub.publish('dfd')
    r.sleep()
