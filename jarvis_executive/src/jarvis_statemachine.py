#!/usr/bin/env python

import roslib; roslib.load_manifest('jarvis_executive')
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from human_intent.msg import Intent
from jarvis_planner.msg import PlanStatus
from jarvis_executive.msg import *


# define the states
class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initiation','armmove'])
        self.counter = 1
        self.transition = 0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state STOP')
        print 'The number of time this state is executing:',self.counter
        pub = rospy.Publisher('Kill', Kill, queue_size=10)
        r = rospy.Rate(10)
        if self.counter > 1:
            self.transition = 0
        
        while not rospy.is_shutdown():
            if self.transition == 0:
                pub.publish(Kill(kill = True))
                rospy.Subscriber('robot_cmd_trial', GoalID, self.userForStop)
            elif self.transition == 1:
                self.transition = -1  # disable the callback after the transition. callback would work if this value = 0
                self.counter += 1     # keeps track of number of state execution and also set self.transition back to 0 
                return 'initiation'
            elif self.transition == 2:
                self.transition = -1
                self.counter += 1
                return 'armmove'  
            r.sleep()
            
    def userForStop(self, userdata):
        """callback for user_interface"""
        if userdata.id == 'come_here' and self.transition == 0:
            print 'Heard "Come here!"'
            self.transition = 1
            
        if userdata.id == 'grab' and self.transition == 0:
            print 'Heard "Grab this!"'
            self.transition = 2


class Basemove(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['basemove_done','basemove_failed'])
        self.counter = 1
        self.timedelay = 0  # keep the time delay for planning upto 10s
        self.plantransition = False  # True iff plan is ready for transition 
        self.transition = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state BASEMOVE')
        print 'The number of time this state is executing:',self.counter
        pubPlan = rospy.Publisher('PlanCommand', PlanCommand, queue_size=10)
        pubCon = rospy.Publisher('Mode', Mode, queue_size=10)
        r = rospy.Rate(10)
        if self.counter > 1:
            self.timedelay = 0
            self.plantransition = False
            self.transition = 0
    
        while not rospy.is_shutdown():
            if self.transition == 0:
                rospy.Subscriber('robot_cmd_trial', GoalID, self.userForBasemove)
                pubPlan.publish(PlanCommand(plancommand = True))
                rospy.Subscriber('PlanStatus_trial', PlanStatus, self.planForBasemove)
                rospy.Subscriber('ControlStatus_trial', String, self.controlforBasemove)
            self.timedelay += 1
            
            elif self.transition == 1:
                self.transition = -1
                pubPlan.publish(PlanCommand(plancommand = False))
                pubCon.publish(Mode(mode = 1))
                self.counter += 1
                return 'basemove_done'
               
            elif self.transition == 2:
                self.transition = -1
                pubPlan.publish(PlanCommand(plancommand = False))
                pubCon.publish(Mode(mode = 3))
                self.counter += 1
                return 'basemove_failed'
               
            elif self.transition == 3:
                self.transition = -1
                pubPlan.publish(PlanCommand(plancommand = False))
                pubCon.publish(Mode(mode = 4))
                return 'basemove_done'
               
            print self.timedelay
            r.sleep()

    def userForBasemove(self, userdata):
        """callback for user_interface"""
        if userdata.id == 'stop' and self.transition == 0:
            print 'Heard "Stop!"'
            self.transition = 1
            
    def planForBasemove(self, userdata):
        """callback for jarvis_planner"""
        if userdata.PlanStatus and self.transition == 0:
            print 'Basemove plan is ready!"'
            pubCon.publish(Mode(mode = 2))
            
        if self.timedelay >= 100 and self.transition == 0:
            print 'Maximum time passed for planning... Plan Failed!'
            self.transition = 2
        
    def controlforBasemove(self, userdata):
        """callback for jarvis_controls"""
        if userdata.data == 'base_control_done' and self.transition == 0:
            print 'base successfully moved!'
            self.transition = 3

'''
class Armmove(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initiation','armmove'])
        self.counter = 0

    def execute(self, userdata):
        pass
        rospy.loginfo('Executing state STOP')
        pub = rospy.Publisher('Kill', Kill, queue_size=10)
        r = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            pub.publish(Kill(kill = True))
            rospy.Subscriber('robot_cmd_trial', GoalID, self.userForStop)
            r.sleep()

    def userForStop(self, userdata):
        """callback for user_interface"""
        if userdata.id == 'come_here':
            print 'Heard "Come here!"'
            return 'initiation'
        elif userdata.id == 'grab':
            print 'Heard "Grab this!"'
            return 'armmove'
        

class Hold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initiation','armmove'])
        self.counter = 0

    def execute(self, userdata):
        pass
        
        rospy.loginfo('Executing state STOP')
        pub = rospy.Publisher('Kill', Kill, queue_size=10)
        r = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            pub.publish(Kill(kill = True))
            rospy.Subscriber('robot_cmd_trial', GoalID, self.userForStop)
            r.sleep()

    def userForStop(self, userdata):
        """callback for user_interface"""
        if userdata.id == 'come_here':
            print 'Heard "Come here!"'
            return 'initiation'
        elif userdata.id == 'grab':
            print 'Heard "Grab this!"'
            return 'armmove'
        

class Adjust(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initiation','armmove'])
        self.counter = 0

    def execute(self, userdata):
        pass
        
        rospy.loginfo('Executing state STOP')
        pub = rospy.Publisher('Kill', Kill, queue_size=10)
        r = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            pub.publish(Kill(kill = True))
            rospy.Subscriber('robot_cmd_trial', GoalID, self.userForStop)
            r.sleep()

    def userForStop(self, userdata):
        """callback for user_interface"""
        if userdata.id == 'come_here':
            print 'Heard "Come here!"'
            return 'initiation'
        elif userdata.id == 'grab':
            print 'Heard "Grab this!"'
            return 'armmove'
        '''


# main
def main():
    rospy.init_node('jarvis_statemachine')

    # Create a SMACH state machine
    #sm = smach.StateMachine(outcomes=['success', 'failure'])
    sm = smach.StateMachine(outcomes=['ARMMOVE', 'failure'])

    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('STOP', Stop(), 
                               transitions={'initiation':'BASEMOVE','armmove':'ARMMOVE'})
        smach.StateMachine.add('BASEMOVE', Basemove(), 
                               transitions={'basemove_done':'STOP','basemove_failed':'failure'})
        '''
        smach.StateMachine.add('ARMMOVE', Armmove(), 
                               transitions={'armmove_done':'HOLD','armmove_failed':'failure', 'armmove_stop':'STOP'})
        smach.StateMachine.add('HOLD', Hold(), 
                               transitions={'hold_failed':'ARMMOVE', 
                               'yes_adjustment':'ADJUST', 'hold_stop':'STOP', 'job_done':'success'})
        smach.StateMachine.add('ADJUST', Adjust(), 
                               transitions={'adjust_stop':'STOP', 'no_adjustment':'HOLD'})
        '''
 
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
