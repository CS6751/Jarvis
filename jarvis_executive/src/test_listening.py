#!/usr/bin/env python
# license from conrell university CS 6751

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from human_intent.msg import Intent
from jarvis_planner.msg import PlanStatus
from jarvis_executive.msg import *

class listener(object):
    
    def __init__(self):
        self.num = 0
        r = rospy.Rate(10)
        rospy.init_node('test_listening', anonymous=True)
        
        while not rospy.is_shutdown():
            #rospy.Subscriber('robot_cmd_trial', GoalID, user_interface_callback)
            #rospy.Subscriber('intents_trial', Intent, human_intent_callback)
            #rospy.Subscriber('PlanStatus_trial', PlanStatus, jarvis_planner_callback)
            rospy.Subscriber('Kill', Kill, callback)
            self.num += 1
            r.sleep()
            
    def callback(self, userdata):
        if userdata.kill:
            print 'listening Kill topic!!!'
            #rospy.loginfo(userdata)
            print self.num
            
if __name__ == '__main__':
    listener()
