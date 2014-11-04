#!/usr/bin/env python
# license from cornell university CS 6751

import rospy
from jarvis_executive.msg import *
from std_msgs.msg import String

def executive_publisher():
    print 'Starting executive_publisher node'
    publish_to = ['Kill', 'Mode', 'PlanCommand']
    pub0 = rospy.Publisher(publish_to[0] , Kill, queue_size=10)
    pub1 = rospy.Publisher(publish_to[1], Mode, queue_size=10)
    pub2 = rospy.Publisher(publish_to[2], PlanCommand, queue_size=10)
    topics = [pub1, pub2, pub3]
    rospy.init_node('executive_publisher', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        str = "executive giving commands %s"%rospy.get_time()
        rospy.loginfo(str)
        pub0.publish(str)
        pub1.publish(str)
        pub2.publish(str)
        r.sleep()
        
if __name__ == '__main__':
    try:
        executive_publisher()
    except rospy.ROSInterruptException: pass
