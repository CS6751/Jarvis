#! /usr/bin/env python

import sys
import rospy
from jarvis_perception import *
import geometry_msgs
from jarvis_perception.srv import *
import jarvis_perception.msg
from visualization_msgs.msg import Marker

def return_grips_client():
    rospy.wait_for_service('return_grips')
    try:
        grip_server = rospy.ServiceProxy('return_grips',ReturnGrips)
        grips = grip_server()
        return grips.grips
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "Takes no arguments, returns an array of poses"

def talker():
    pub = rospy.Publisher('grip_viz', Marker, queue_size = 10)
    rospy.init_node('grip_viz_publisher', anonymous = True)
    rate = rospy.Rate(1)
    grip_num = 0;
    while not rospy.is_shutdown():
        grips = return_grips_client()
        marker = Marker()
        marker.header.frame_id = "/board_tf"
        marker.type = marker.CUBE
        marker.pose.position = grips.grasps[grip_num].point
        marker.pose.orientation = grips.grasps[grip_num].orientation
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.1

        marker.color.a = 1.0
        pub.publish(marker)
        if grip_num < 3:
            grip_num += 1
        else:
            grip_num = 0
        rate.sleep()


if __name__ == "__main__":
    grips = return_grips_client()
    print "Requesting grips"
    print type(grips)
    print type(grips.grasps[3])
    print "grip2 quaternion = " +str(grips.grasps[1].orientation)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
