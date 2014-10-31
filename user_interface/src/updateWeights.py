#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import user_interface.msg
import math


def updateWeights(grips):
    newWeight1 = 0.3
    newWeight2 = 0.7

    point1 = grips[0].Point
    quat1 = grips[0].Quaternion
    weight1 = newWeight1 * grips[0].Weight
    grip1 = user_interface.msg.GraspBox(point1,quat1,weight1)

    point2 = grips[1].Point
    quat2 = grips[1].Quaternion
    weight2 = newWeight2 * grips[1].Weight
    grip2 = user_interface.msg.GraspBox(point2,quat2,weight2)

    newgrips = user_interface.msg.GraspArray()
    newgrips.grasps.append(grip1)
    newgrips.grasps.append(grip2)

    newgrips.header.frame_id = '1'

    return newgrips

if __name__ == "__main__":
     updateWeights()
