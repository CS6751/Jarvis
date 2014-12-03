#!/usr/bin/env python
# license from cornell university CS 6751

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID

def user_interface_pub():
    pub = rospy.Publisher('robot_cmd', GoalID, queue_size=10)
    rospy.init_node('user_interface_pub', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        goalid = '3'
        rospy.loginfo(goalid)
        pub.publish(goalid)
        r.sleep()
        
if __name__ == '__main__':
    try:
        user_interface_pub()
    except rospy.ROSInterruptException: pass
