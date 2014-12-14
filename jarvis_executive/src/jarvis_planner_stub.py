#!/usr/bin/env python
# license from cornell university CS 6751

import rospy
from std_msgs.msg import String
from jarvis_planner.msg import PlanStatus
from jarvis_executive.msg import *

class jarvis_planner_pub(object):
    
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.pub = rospy.Publisher('PlanStatus_trial', PlanStatus, queue_size=10)
        r = rospy.Rate(10) # 10hz
        self.transition = 0
        self.num = 0
        
        while not rospy.is_shutdown():
            if self.transition == 0:
                rospy.Subscriber('PlanCommand', PlanCommand, self.callback)
             
            if self.transition == -1:
                self.transition = 0
                self.num = 0
             
            if self.transition == 1:
                print 'Plan Ready!'
                self.pub.publish(True)
                self.transition = -1
                
            r.sleep()
        
    def callback(self, userdata):
        if userdata.plancommand and self.transition == 0:
            #print 'begin planning for basemove'
            self.num += 1
            #print self.num
            if self.num >= 2000:
                self.transition = 1
                
    def cleanup(self):
        print 'plan quitting...'
        self.pub.publish(False)
                
       
if __name__ == '__main__':
    rospy.init_node('jarvis_planner_stub', anonymous=True)
    try:
        jarvis_planner_pub()
    except rospy.ROSInterruptException: pass
