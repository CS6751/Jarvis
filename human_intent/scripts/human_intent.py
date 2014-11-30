#!/usr/bin/env python

import rospy
from Intent.msg import intent

def human_intent():
    pub = rospy.Publisher('intents', intent, queue_size=10)
    rospy.init_node('human_intent', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        inte = intent()
     	# inte = "sdafsada"
        pub.publish(inte)
        r.sleep()
        
if __name__ == '__main__':
    try:
        human_intent()
    except rospy.ROSInterruptException: pass
