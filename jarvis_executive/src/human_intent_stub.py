#!/usr/bin/env python
# license from cornell university CS 6751

import rospy
from std_msgs.msg import String
from human_intent.msg import Intent
from jarvis_executive.msg import *

def human_intent_pub():
    pub = rospy.Publisher('intents_trial', Intent, queue_size=10)
    rospy.init_node('human_intent_stub', anonymous=True)
    r = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        intent_num = 1  # for Hold
        #intent_num = 2  # for Adjust
        rospy.loginfo(intent_num)
        pub.publish(intent_num)
        
        r.sleep()

if __name__ == '__main__':
    try:
        human_intent_pub()
    except rospy.ROSInterruptException: pass
