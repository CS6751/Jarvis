#! /usr/bin/env python

import sys
import rospy
from jarvis_perception import *
import geometry_msgs
from jarvis_perception.srv import *
import jarvis_perception.msg
import updateWeights

def return_grips_client():
    rospy.wait_for_service('return_grips')
    try:
        grip_server = rospy.ServiceProxy('return_grips',ReturnGrips)
        grips = grip_server()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    #now, update the weights and publish the result
    try:
        pub = rospy.Publisher('chatter', jarvis_perception.msg.GraspArray, queue_size=10)
        rospy.init_node('UI_grip_pub', anonymous=True)
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            gripsWithUpdatedWeights = updateWeights.updateWeights(grips)
            rospy.loginfo(gripsWithUpdatedWeights)
            pub.publish(gripsWithUpdatedWeights)
            return grips.grips
            r.sleep()
    except rospy.ROSInterruptException: pass 

def usage():
    return "requests grip positions/weights from the perception module"

if __name__ == "__main__":
    grips = return_grips_client()
    print "Requesting grips"
    print type(grips)
    print type(grips.grasps[1])
    print "grip2 quaternion = " +str(grips.grasps[1].orientation)
