#!/usr/bin/env python
# license from conrell university CS 6751

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from human_intent.msg import Intent
from jarvis_planner.msg import PlanStatus
from jarvis_executive.msg import *


def callback(userdata):
    if userdata.kill:
        print 'listening Kill topic!!!'
        #rospy.loginfo(userdata)
    	print num
    	
def listener():
    num = 0
    main()
    
def main():
    rospy.init_node('test_listening', anonymous=True)
    #rospy.Subscriber('robot_cmd_trial', GoalID, user_interface_callback)
    #rospy.Subscriber('intents_trial', Intent, human_intent_callback)
    #rospy.Subscriber('PlanStatus_trial', PlanStatus, jarvis_planner_callback)
    rospy.Subscriber('Kill', Kill, callback)
    num += 1
    rospy.spin()
    
if __name__ == '__main__':
    listener()
