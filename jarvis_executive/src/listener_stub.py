#!/usr/bin/env python
# license from conrell university CS 6751

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from human_intent.msg import Intent
from jarvis_planner.msg import PlanStatus


def user_interface_callback(userdata):
    if userdata.id== '10':
      print 'correct!!!'
    else:
      print 'go away..'

def human_intent_callback(userdata):
    if userdata.intent == 1:
        print 'intent Hold!'
    else:
	    print 'intent Adjust!'

def jarvis_planner_callback(userdata):
    if userdata.PlanStatus == True:
    	print 'plan ready!'
    else:
    	print 'wait for planning...'

def listener():

    rospy.init_node('listener_stub', anonymous=True)

    rospy.Subscriber('robot_cmd_trial', GoalID, user_interface_callback)
    rospy.Subscriber('intents_trial', Intent, human_intent_callback)
    rospy.Subscriber('PlanStatus_trial', PlanStatus, jarvis_planner_callback)
    
    rospy.spin()
        
if __name__ == '__main__':
    listener()
