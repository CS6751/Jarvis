#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import jarvis_perception.msg
import math


def findGrips():
    point1 = geometry_msgs.msg.Point(0.0,0.0,0.0)
    quat1 = geometry_msgs.msg.Quaternion(0,0,0,1)
    weight1 = 10.0 
    grip1 = jarvis_perception.msg.GraspBox(point1,quat1,weight1)

    point2 = geometry_msgs.msg.Point(1,2,3)
    quat2 = geometry_msgs.msg.Quaternion(math.sin(math.pi/4),0,0,math.cos(math.pi/4))
    weight2 = 90.0
    grip2 = jarvis_perception.msg.GraspBox(point2,quat2,weight2)

    grips = jarvis_perception.msg.GraspArray()
    grips.grasps.append(grip1)
    grips.grasps.append(grip2)

    grips.header.frame_id = '1'

    return grips

if __name__ == "__main__":
     findGrips()
