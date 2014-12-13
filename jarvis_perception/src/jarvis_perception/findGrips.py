#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import jarvis_perception.msg
import math
from tf.transformations import quaternion_from_euler

BOARD_X = 0.0015;
BOARD_Y = 0.1015;
BOARD_Z = 0.1395;
CENTER_X = 0.015;
CENTER_Y = BOARD_Y/2-0.01;
CENTER_Z = 0.025-BOARD_Z/2;
NUM_GRIPS = 4;
ROLLS =[90,180,180,-90];
PITCHES=[0,0,0,0];
YAWS=[0,0,0,0];
X=[CENTER_X,CENTER_X,CENTER_X,CENTER_X];
Y=[CENTER_Y-BOARD_Y/2,CENTER_Y-BOARD_Y/6,
    CENTER_Y+BOARD_Y/6, CENTER_Y+BOARD_Y/2];
Z=[CENTER_Z, CENTER_Z-BOARD_Z/2,
    CENTER_Z-BOARD_Z/2, CENTER_Z];
WEIGHTS = [0.1, 0.5, 0.5, 0.9];

def findGrips():
    grips = jarvis_perception.msg.GraspArray()
    grips.header.frame_id = '/board_tf'
    for i in range(0, 4):
        p = geometry_msgs.msg.Point(X[i],Y[i],Z[i])
        q = quaternion_from_euler(ROLLS[i],PITCHES[i],YAWS[i])
        q2 = geometry_msgs.msg.Quaternion(q[0],q[1],q[2],q[3])
        grip = jarvis_perception.msg.GraspBox(
                p,q2,WEIGHTS[i])
        grips.grasps.append(grip)

    return grips

if __name__ == "__main__":
     findGrips()
