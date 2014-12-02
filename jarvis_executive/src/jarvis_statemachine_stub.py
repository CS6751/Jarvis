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
    topics = [pub0, pub1, pub2]
    rospy.init_node('executive_publisher', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        str = "executive giving commands %s"%rospy.get_time()
        rospy.loginfo(str)
        msg0 = Kill()
        msg1 = Mode()
        msg2 = PlanCommand()
        pub0.publish(msg0)
        pub1.publish(msg1)
        pub2.publish(msg2)
        r.sleep()
        
if __name__ == '__main__':
    try:
        executive_publisher()
    except rospy.ROSInterruptException: pass
