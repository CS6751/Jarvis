#!/usr/bin/env python

import rospy
from jarvis_perception.msg import AxisAlignedBox
from jarvis_perception import get_box


def object_publisher():
    rospy.init_node('object_publisher')
    print 'Starting object_publisher node'
    objects = ['target','hand','finger']
    topics = []
    for obj in objects:
        pub = rospy.Publisher('objects/'+obj, AxisAlignedBox,queue_size=10)
        topics.append(pub)
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        for idx, val in enumerate(objects):
            box = get_box.get_box(objects[idx],0)
           # rospy.loginfo(box)
            topics[idx].publish(box)
            r.sleep()

if __name__=="__main__":
    object_publisher()
