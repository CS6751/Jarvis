#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from jarvis_perception.msg import GraspBox
from jarvis_perception.msg import GraspArray
import math


def updateWeights(grips, utterance, AxisAlignedBox):
    newWeight1 = 0.3
    newWeight2 = 0.7

    oldGrips = grips.grips.grasps
    # print "%s"%oldGrips[0]

    weight1 = newWeight1 * oldGrips[0].weight
    grip1 = GraspBox(oldGrips[0].point,oldGrips[0].orientation,weight1)

    weight2 = newWeight2 * oldGrips[1].weight
    grip2 = GraspBox(oldGrips[1].point,oldGrips[1].orientation,weight2)

    newgrips = GraspArray()
    newgrips.grasps.append(grip1)
    newgrips.grasps.append(grip2)

    newgrips.header.frame_id = '1'

    return newgrips

if __name__ == "__main__":
     updateWeights()
