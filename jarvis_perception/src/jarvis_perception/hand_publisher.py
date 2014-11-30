#!usr/bin/env python

import rospy
from jarvis_perception.srv import AxisAlignedBox
from jarvis_perception import get_box

def object_publisher(name):
    pub = rospy.Publisher('objects/'+name, AxisAlignedBox, queue_size = 10)
    rospy.init_node(name + '_publisher'
