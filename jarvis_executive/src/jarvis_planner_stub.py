#!/usr/bin/env python
# license from cornell university CS 6751

import rospy
from std_msgs.msg import String
from jarvis_planner.msg import PlanStatus

def jarvis_planner_pub():
    pub = rospy.Publisher('PlanStatus_trial', PlanStatus, queue_size=10)
    rospy.init_node('jarvis_planner_stub', anonymous=True)
    r = rospy.Rate(10) # 10hz
    
    counter = 0
    boolean = [True, False]
    while not rospy.is_shutdown():
        if counter == 50:
          plan_ready = boolean[0] 
        else:
          plan_ready = boolean[1]
         
        rospy.loginfo(plan_ready)
        pub.publish(plan_ready)
        counter += 1
        r.sleep()

if __name__ == '__main__':
    try:
        jarvis_planner_pub()
    except rospy.ROSInterruptException: pass
