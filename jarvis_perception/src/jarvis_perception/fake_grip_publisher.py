#! /usr/bin/env python

import rospy
import geometry_msgs
from geometry_msgs.msg import PoseStamped

def publisher():
    pub = rospy.Publisher('grasps', PoseStamped, queue_size=10)
    rospy.init_node('fake_grip_publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        grip = geometry_msgs.msg.PoseStamped()
        grip.header.frame_id = "target"
        grip.pose.position = geometry_msgs.msg.Point(0,0,0)
        grip.pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,1)
        pub.publish(grip)
        rate.sleep()



if __name__=='__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
