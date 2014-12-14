#!/usr/bin/env python
# license from cornell university CS 6751

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from jarvis_executive.msg import *

class  user_interface_pub(object):
    
    def __init__(self):
        print 'Start user_interface!'
        rospy.on_shutdown(self.cleanup)
        self.pub = rospy.Publisher('robot_cmd_trial', GoalID, queue_size=10)
        r = rospy.Rate(5) # 10hz
        #num = 0
        
        self.msg = GoalID()
        rospy.Subscriber('recognizer/output', String, self.speechCb)
        
        while not rospy.is_shutdown():
            #data = GoalID(id = str(num))
            #rospy.loginfo(self.msg)
            #pub.publish(self.msg)
            self.pub.publish(self.msg)
            r.sleep()
        
    def speechCb(self, userdata):
        rospy.loginfo(userdata.data)
        
        if userdata.data.find('come here') > -1:
            print 'heard come here!'
            self.msg.id = 'come_here'
        
        elif userdata.data.find('grab') > -1:
            print 'heard grab!'
            self.msg.id = 'grab'
    
        elif userdata.data.find('stop') > -1:
            print 'heard stop!'
            self.msg.id = 'stop' 
            
        elif userdata.data.find('done') > -1:
            print 'heard done!'
            self.msg.id = 'done' 
            
        elif userdata.data.find('good job') > -1:
            print 'heard good job!'
            self.msg.id = 'good_job' 
            
        elif userdata.data.find('adjust') > -1:
            print 'heard adjust!'
            self.msg.id = 'adjust'
            
        elif userdata.data.find('hold') > -1:
            print 'heard hold!'
            self.msg.id = 'hold' 
            
    
    def cleanup(self):
        print 'user interface exiting...'
        command = GoalID()
        rospy.info(command)
        self.pub.publish(command)
        
        '''
        if num == 20:
            toBasemove = GoalID(id = 'come_here')
            pub.publish(toBasemove)
            print 'move to base move transition!!!!!!!'    
        
        #if num == 21:
        #    m = GoalID(id = None)
        #    pub.publish(m)
        
        
        if num == 30:
            to = GoalID(id = 'grab')
            pub.publish(to)
            print 'move to grab!!!!!!!'
            
        #if num == 31:
        #    tt = GoalID(id = None)
        #    pub.publish(tt)

        if num == 50:
            stop = GoalID(id = 'stop')
            pub.publish(stop)
            
        if num == 70:
            toBasemove = GoalID(id = 'come_here')
            pub.publish(toBasemove)
            print 'move to base move 2nd time!!!!!!!'
            
            
        if num == 80:
            to = GoalID(id = 'grab')
            pub.publish(to)
            print 'move to grab!!!!!!!'
            
        if num == 90:
            stop = GoalID(id = 'stop')
            pub.publish(stop)  
            
        if num == 100:
            to = GoalID(id = 'grab')
            pub.publish(to)
            print 'move to grab!!!!!!!'    
        
        
        num = num+1
        r.sleep()
        '''
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
    rospy.init_node('user_interface_stub', anonymous=True)
    try:
        user_interface_pub()
    except rospy.ROSInterruptException: pass
