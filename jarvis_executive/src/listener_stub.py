#!/usr/bin/env python
# license from conrell university CS 6751

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from human_intent.msg import Intent


def user_interface_callback(userdata):
    if userdata.data == '3':
      print 'correct!!!'
    else:
      print 'go away..'

def human_intent_callback(userdata):
    if userdata.intent == 1:
        print 'intent correct!'

def listener():

    rospy.init_node('listener_stub', anonymous=True)

    rospy.Subscriber('robot_cmd_trial', String, user_interface_callback)
    rospy.Subscriber('intents_trial',Intent, human_intent_callback)
    
    rospy.spin()
        
if __name__ == '__main__':
    listener()
