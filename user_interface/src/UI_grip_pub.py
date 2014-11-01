#!/usr/bin/env python

import rospy
from jarvis_perception import GraspArray
import updateWeights

def publish_grip_loc():
    pub = rospy.Publisher('chatter', GraspArray.msg, queue_size=10)
    rospy.init_node('UI_grip_pub', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        gripsWithUpdatedWeights = updateWeights.updateWeights(grips) # how to bring in grips from client??
        rospy.loginfo(gripsWithUpdatedWeights)
        pub.publish(gripsWithUpdatedWeights)
        r.sleep()
        
if __name__ == '__main__':
    try:
        publish_grip_loc()
    except rospy.ROSInterruptException: pass
