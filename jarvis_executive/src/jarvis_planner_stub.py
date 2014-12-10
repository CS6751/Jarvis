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
        self.counter = 0
        self.num = 0
    
        while not rospy.is_shutdown():
            rospy.Subscriber('PlanCommand', PlanCommand, self.callback)
            print self.counter
            self.counter += 1
            r.sleep()
        
    def callback(self, userdata):
        if userdata.plancommand:
            print 'begin planning for basemove'
            self.num += 1
            print self.num
            if self.num >= 5:
                print 'plan ready'
                pub.publish(True)
        else:
            pub.publish(False)
        

if __name__ == '__main__':
    rospy.init_node('jarvis_planner_stub', anonymous=True)
    try:
        jarvis_planner_pub()
    except rospy.ROSInterruptException: pass
