#!/usr/bin/env python

import rospy
from actionlib_msgs import GoalID

def publish_command():
    pub = rospy.Publisher('chatter', GoalID, queue_size=10)
    rospy.init_node('UI_command_pub', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()
        
if __name__ == '__main__':
    try:
        publish_command()
    except rospy.ROSInterruptException: pass
