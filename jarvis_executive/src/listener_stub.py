#!/usr/bin/env python
# license from conrell university CS 6751

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID


def callback(userdata):
    if userdata.data == '3':
      print 'correct!!!'
    else:
      print 'go away..'


def listener():

    rospy.init_node('listener_stub', anonymous=True)

    rospy.Subscriber('robot_cmd_trial', String, callback)

    
    rospy.spin()
        
if __name__ == '__main__':
    listener()
