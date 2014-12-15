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
        self.timedelay = 0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state STOP')
        print 'The number of time this state is executing:',self.counter
        pub = rospy.Publisher('Kill', Kill, queue_size=10)
        r = rospy.Rate(10)
        if self.counter > 1:
            self.transition = 0
            self.timedelay = 0
        
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
            self.timedelay += 1   
            print self.counter, 'STOP ( time:',self.timedelay,')'
            r.sleep()
            
    def userForStop(self, userdata):
        """callback for user_interface"""
        if userdata.id == 'come_here' and self.transition == 0:
            print 'Heard "Come here!"'
            self.transition = 1
            
        if userdata.id == 'grab' and self.transition == 0:
            print 'Heard "Grab!"'
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
            # Initial case
            if self.transition == 0:
                rospy.Subscriber('robot_cmd_trial', GoalID, self.userForBasemove)
                pubPlan.publish(PlanCommand(plancommand = True))
                rospy.Subscriber('PlanStatus_trial', PlanStatus, self.planForBasemove)
            if (self.timedelay >= 30) and self.transition == 0: 
                    print 'Maximum time passed for planning... Plan Failed!'
                    self.transition = 3
              
            # when heard "Stop!"   
            elif self.transition == 1:
                self.transition = -1
                pubPlan.publish(PlanCommand(plancommand = False))
                pubCon.publish(Mode(mode = 1))
                self.counter += 1
                return 'basemove_done'
               
            # when plan is ready to execute
            elif self.transition == 2:
                rospy.Subscriber('robot_cmd_trial', GoalID, self.userForBasemove)
                pubPlan.publish(PlanCommand(plancommand = False))
                rospy.Subscriber('ControlStatus_trial', String, self.controlforBasemove)
                pubCon.publish(Mode(mode = 2))
            
            # when plan fails
            elif self.transition == 3: 
                self.transition = -1
                pubPlan.publish(PlanCommand(plancommand = False))
                pubCon.publish(Mode(mode = 3))
                self.counter += 1
                return 'basemove_failed'
            
            # base finishes moving
            elif self.transition == 4:
                self.transition = -1
                pubCon.publish(Mode(mode = 4))
                self.counter += 1
                return 'basemove_done'
            self.timedelay += 1   
            print self.counter, 'BASEMOVE ( time:',self.timedelay,')' 
            r.sleep()

    def userForBasemove(self, userdata):
        """callback for user_interface"""
        if userdata.id == 'stop' and (self.transition == 0 or self.transition == 2):
            print 'Heard "Stop!"'
            self.transition = 1
            
    def planForBasemove(self, userdata):
        """callback for jarvis_planner"""
        if userdata.PlanStatus and self.transition == 0:
            print 'Basemove plan is ready!"'
            self.transition = 2
        
    def controlforBasemove(self, userdata):
        """callback for jarvis_controls"""
        if userdata.data == 'base_control_done' and self.transition == 2:
            print 'base successfully moved!'
            self.transition = 4


class Armmove(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['armmove_stop','armmove_failed','armmove_done'])
        self.counter = 1
        self.timedelay = 0
        self.plantransition = False  # True iff plan is ready for transition 
        self.transition = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state ARMMOVE')
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
                rospy.Subscriber('robot_cmd_trial', GoalID, self.userForArmmove)
                pubPlan.publish(PlanCommand(plancommand = True))
                rospy.Subscriber('PlanStatus_trial', PlanStatus, self.planForArmmove)

            elif self.transition == 1:
                self.transition = -1
                pubPlan.publish(PlanCommand(plancommand = False))
                pubCon.publish(Mode(mode = 1))
                self.counter += 1
                return 'armmove_stop'
               
            elif self.transition == 2:
                rospy.Subscriber('robot_cmd_trial', GoalID, self.userForArmmove)
                pubPlan.publish(PlanCommand(plancommand = False))
                #rospy.Subscriber('ControlStatus_trial', String, self.controlforArmmove)   # resumes when control works
                pubCon.publish(Mode(mode = 2))
                
            elif self.transition == 3: 
                self.transition = -1
                pubPlan.publish(PlanCommand(plancommand = False))
                pubCon.publish(Mode(mode = 3))
                self.counter += 1
                return 'armmove_failed'
            
            elif self.transition == 4:
                self.transition = -1
                pubPlan.publish(PlanCommand(plancommand = False))
                pubCon.publish(Mode(mode = 4))
                self.counter += 1
                return 'armmove_done'   
            
            self.timedelay += 1   
            print self.counter, 'ARMMOVE ( time:',self.timedelay,')' 
            
            # The following tells which transition is on 
            if self.transition == 0:
                print 'Plan Ready?'
                
            if self.transition == 1:
                print 'Heard "Stop!"'
                
            if self.transition == 2:
                print 'Plan Ready!'
                
            if self.transition == 3:
                print 'Failure!'
                
            if self.transition == 4:
                print 'Arm Moved!'
            
            r.sleep()

    def userForArmmove(self, userdata):
        """callback for user_interface"""
        if userdata.id == 'stop' and (self.transition == 0 or self.transition == 2):
            print 'Heard "Stop!"'
            self.transition = 1

    ### the following 3 line must be removed when control subscriber is ready
        if userdata.id == 'done' and (self.transition == 2):
            print 'arm successfully moved!'
            self.transition = 4 
    ### the above 3 line must be removed when control subscriber is ready
            
    def planForArmmove(self, userdata):
        """callback for jarvis_planner"""
        if (self.counter == 3) and (self.transition == 0):  # armmove performing 3 times is considered failure
            print 'ARMMOVE State reached maximum number of time... Plan Failed!'
            self.transition = 3
            
        if userdata.PlanStatus and self.transition == 0:
            print 'Armmove plan is ready!"'
            self.transition = 2
    '''    
    def controlforArmmove(self, userdata):
        """callback for jarvis_controls"""
        if userdata.data == 'arm_control_done' and self.transition == 2:
            print 'arm successfully moved!'
            self.transition = 4
    '''
        

class Hold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hold_failed','hold_stop','job_done','yes_adjustment'])
        self.counter = 1
        self.transition = 0
        self.timedelay = 0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state HOLD')
        print 'The number of time this state is executing:',self.counter
        pubCon = rospy.Publisher('Mode', Mode, queue_size=10)
        pubCon.publish(Mode(mode = 5))

        r = rospy.Rate(10)
        if self.counter > 1:
            self.transition = 0
            self.timedelay = 0
        
        while not rospy.is_shutdown():
            if self.transition == 0:
                pubCon.publish(Mode(mode = 5))
                rospy.Subscriber('robot_cmd_trial', GoalID, self.userForStop)
            
            elif self.transition == 1:
                self.transition = -1  # disable the callback after the transition. callback would work if this value = 0
                self.counter += 1     # keeps track of number of state execution and also set self.transition back to 0 
                return 'hold_stop'
            
            elif self.transition == 2:
                self.transition = -1
                self.counter += 1
                return 'hold_failed' 
                
            elif self.transition == 3:
                self.transition = -1
                self.counter += 1
                return 'yes_adjustment'
                
            elif self.transition == 4:
                self.transition = -1
                self.counter += 1
                return 'job_done'    
                
            self.timedelay += 1   
            print self.counter, 'HOLD ( time:',self.timedelay,')'
            r.sleep()
            
    def userForStop(self, userdata):
        """callback for user_interface"""
        if userdata.id == 'stop' and self.transition == 0:
            print 'Heard "Stop!"'
            self.transition = 1
            
        if userdata.id == 'grab' and self.transition == 0:
            print 'Heard "Grab!"'
            self.transition = 2
            
        if userdata.id == 'adjust' and self.transition == 0:
            print 'Heard "Adjust!"'
            self.transition = 3
        
        if userdata.id == 'good_job' and self.transition == 0:
            print 'Heard "Good job!"'
            self.transition = 4


class Adjust(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['adjust_stop','no_adjustment'])
        self.counter = 1
        self.transition = 0
        self.timedelay = 0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state ADJUST')
        print 'The number of time this state is executing:',self.counter
        pubCon = rospy.Publisher('Mode', Mode, queue_size=10)
        pubCon.publish(Mode(mode = 6))

        r = rospy.Rate(10)
        if self.counter > 1:
            self.transition = 0
            self.timedelay = 0
        
        while not rospy.is_shutdown():
            if self.transition == 0:
                pubCon.publish(Mode(mode = 6))
                rospy.Subscriber('robot_cmd_trial', GoalID, self.userForStop)
            
            elif self.transition == 1:
                self.transition = -1  # disable the callback after the transition. callback would work if this value = 0
                self.counter += 1     # keeps track of number of state execution and also set self.transition back to 0 
                return 'adjust_stop'
            
            elif self.transition == 2:
                self.transition = -1
                self.counter += 1
                return 'no_adjustment' 
                
            self.timedelay += 1   
            print self.counter, 'ADJUST ( time:',self.timedelay,')'
            r.sleep()
            
    def userForStop(self, userdata):
        """callback for user_interface"""
        if userdata.id == 'stop' and self.transition == 0:
            print 'Heard "Stop!"'
            self.transition = 1
            
        if userdata.id == 'hold' and self.transition == 0:
            print 'Heard "Hold!"'
            self.transition = 2
            
            

# main
def main():
    rospy.init_node('jarvis_statemachine')

    # Create a SMACH state machine
    #sm = smach.StateMachine(outcomes=['success', 'failure'])
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('STOP', Stop(), 
                               transitions={'initiation':'BASEMOVE','armmove':'ARMMOVE'})
        
        smach.StateMachine.add('BASEMOVE', Basemove(), 
                               transitions={'basemove_done':'STOP','basemove_failed':'failure'})
        
        smach.StateMachine.add('ARMMOVE', Armmove(), 
                               transitions={'armmove_done':'HOLD','armmove_failed':'failure', 'armmove_stop':'STOP'})
        
        smach.StateMachine.add('HOLD', Hold(), 
                               transitions={'hold_failed':'ARMMOVE', 
                               'yes_adjustment':'ADJUST', 'hold_stop':'STOP', 'job_done':'success'})
        
        smach.StateMachine.add('ADJUST', Adjust(), 
                               transitions={'adjust_stop':'STOP', 'no_adjustment':'HOLD'})
        
 
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
 
    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass