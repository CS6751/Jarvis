#!/usr/bin/env python
# license from cornell university CS 6751

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID

def user_interface_pub():
    pub = rospy.Publisher('robot_cmd_trial', GoalID, queue_size=10)
    rospy.init_node('user_interface_stub', anonymous=True)
    r = rospy.Rate(10) # 10hz
    num = 0
    while not rospy.is_shutdown():
        data = GoalID(id = str(num))
        #time_stamp = str(rospy.get_time())
        #num = 3
        #string_id = id(num)
        #data = GoalID(time_stamp,string_id) 
        #rospy.loginfo(time_stamp+' command is < %s >',string_id)
        #pub.publish(time_stamp, string_id)
        #pub.publish('0', string_id)
        rospy.loginfo(data)
        pub.publish(data)
        
        if num == 20:
            toBasemove = GoalID(id = 'come_here')
            pub.publish(toBasemove)
            
        if num == 100:
            stop = GoalID(id = 'stop')
            pub.publish(stop)
        num = num+1
        r.sleep()
'''

def user_interface_pub():
    pub = rospy.Publisher('robot_cmd_trial', String, queue_size=10)
    rospy.init_node('user_interface_stub', anonymous=True)
    r = rospy.Rate(10) # 10hz
    num = 0
    while not rospy.is_shutdown():
        string_id = str(num)
        #pub.publish(time_stamp, string_id)
        #pub.publish('0', string_id)
        rospy.loginfo(string_id)
        pub.publish(string_id)
        num = num+1
        r.sleep()
'''
if __name__ == '__main__':
    try:
        user_interface_pub()
    except rospy.ROSInterruptException: pass
