#!/usr/bin/env python
# license from cornell university CS 6751

import rospy
from std_msgs.msg import String
from jarvis_planner.msg import PlanStatus
from jarvis_executive.msg import *

class jarvis_planner_pub(object):
    
    def __init__(self):
        pub = rospy.Publisher('PlanStatus_trial', PlanStatus, queue_size=10)
        r = rospy.Rate(1) # 1hz
        self.num = 0
        self.transition = 0
    
        while not rospy.is_shutdown():
            if self.transition == 0:
                rospy.Subscriber('PlanCommand', PlanCommand, self.callback)
             
            if self.transition == 1:
                print 'base plan ready'
                self.transition = 0
                pub.publish(True)
                
            r.sleep()
        
    def callback(self, userdata):
        if userdata.plancommand and self.transition == 0:
            print 'begin planning for basemove'
            self.num += 1
            print self.num
            if self.num >= 80:
                self.transition = 1
                
       
if __name__ == '__main__':
    rospy.init_node('jarvis_planner_stub', anonymous=True)
    try:
        jarvis_planner_pub()
    except rospy.ROSInterruptException: pass
