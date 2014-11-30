#!/usr/bin/env python
import rospy
import math
import jarvis_perception.msg
import geometry_msgs.msg

# This is a stub file that generates a random AxisAlignedBox()
def get_box(name, use_image_processing):
    if (use_image_processing == 1):
    #Do some image processing shenanigans to get the bounding box
        print "processing images"
    else:
    #Make a canned respose
        point2 = geometry_msgs.msg.Point(1,2,3)
        quat2 = geometry_msgs.msg.Quaternion(math.sin(math.pi/4),0,0,math.cos(math.pi/4))
        box = jarvis_perception.msg.AxisAlignedBox()
        box.name = name
        box.w = 0.10
        box.l = 0.15
        box.h = 0.002

        box.pose.position = point2
        box.pose.orientation = quat2
        return box

if __name__ == "__main__":
    get_box(argv[1],argv[2])
