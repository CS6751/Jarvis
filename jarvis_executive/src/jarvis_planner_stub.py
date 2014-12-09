#!/usr/bin/env python
# license from cornell university CS 6751

import rospy
from std_msgs.msg import String
from jarvis_planner.msg import PlanStatus

def jarvis_planner_pub():
    pub = rospy.Publisher('PlanStatus_trial', PlanStatus, queue_size=10)
    rospy.init_node('jarvis_planner_stub', anonymous=True)
    r = rospy.Rate(1) # 10hz
    
    counter = 0
    num = 0
    while not rospy.is_shutdown():
        rospy.Subscriber('PlanCommand', PlanCommand, callback)
        
         
        print counter
        counter += 1
        r.sleep()
        
def callback(userdata):
    if userdata.plancommand:
        print 'begin planning for basemove'
        num += 1
        if num >= 5:
            pub.publish(True)
    else:
        pub.publish(False)
        

if __name__ == '__main__':
    try:
        jarvis_planner_pub()
    except rospy.ROSInterruptException: pass
