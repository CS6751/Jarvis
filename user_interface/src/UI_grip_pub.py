#!/usr/bin/env python

import rospy
import geometry_msgs
import updateWeights

def publish_grip_loc():
    gripsWithUpdatedWeights = updateWeights.updateWeights(req)
    pub = rospy.Publisher('chatter', GraspArray.msgs, queue_size=10)
    rospy.init_node('UI_grip_pub', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()
        
if __name__ == '__main__':
    try:
        publish_grip_loc()
    except rospy.ROSInterruptException: pass
