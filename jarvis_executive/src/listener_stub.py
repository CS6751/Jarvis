#!/usr/bin/env python
# license from conrell university CS 6751

import rospy
from std_msgs.msg import String

def callback(data):
    if data.id == '3'
      print 'correct!!!'
    else:
      print 'go away..'
    
def listener():

    rospy.init_node('listener_stub', anonymous=True)

    rospy.Subscriber('robot_cmd', GoalID, callback)

    
    rospy.spin()
        
if __name__ == '__main__':
    listener()
