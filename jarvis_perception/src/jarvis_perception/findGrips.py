#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import jarvis_perception
import math


def findGrips():
    point1 = geometry_msgs.msg.Point(0.0,0.0,0.0)
    quat1 = geometry_msgs.msg.Quaternion(0,0,0,1)
    grip1 = geometry_msgs.msg.Pose(point1,quat1)

    point2 = geometry_msgs.msg.Point(1,2,3)
    quat2 = geometry_msgs.msg.Quaternion(math.sin(math.pi/4),0,0,math.cos(math.pi/4))
    grip2 = geometry_msgs.msg.Pose(point2,quat2)

    grips = geometry_msgs.msg.PoseArray()
    grips.poses.append(grip1)
    grips.poses.append(grip2)

    grips.header.frame_id = '1'

    return grips

if __name__ == "__main__":
     findGrips()
