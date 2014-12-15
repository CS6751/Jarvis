#! /usr/bin/env python

import sys
import rospy
from jarvis_perception import *
import geometry_msgs
from jarvis_perception.srv import *
import jarvis_perception.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray

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
    pub = rospy.Publisher('grip_viz', PoseArray, queue_size = 10)
    rospy.init_node('grip_viz_publisher', anonymous = True)
    rate = rospy.Rate(10)
    grip_num = 0;
    while not rospy.is_shutdown():
        poses = geometry_msgs.msg.PoseArray()
        grips = return_grips_client()
        
        poses.header.frame_id = grips.header.frame_id
        for i in range(0,len(grips.grasps)):
            pose = geometry_msgs.msg.Pose()
            pose.position = grips.grasps[i].point
            pose.orientation = grips.grasps[i].orientation
            poses.poses.append(pose)
        pub.publish(poses)
        rate.sleep()


if __name__ == "__main__":
    grips = return_grips_client()
    print "Requesting grips"
    print "Initial number of grasps"
    print len(grips.grasps)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
