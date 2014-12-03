#!/usr/bin/env python

import roslib; roslib.load_manifest('jarvis_executive')
import rospy
import smach
import smach_ros

# define state Foo
class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initiation','waiting','armmove'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state STOP')
       
        if self.counter < 10:
            self.counter += 1
            return 'waiting'
        else:
            return 'armmove'




# main
def main():
    rospy.init_node('jarvis_statemachine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('STOP', Stop(), 
                               transitions={'initiation':'BASEMOVE', 'waiting':'STOP', 'armmove':'ARMMOVE'})
        smach.StateMachine.add('BASEMOVE', Basemove(), 
                               transitions={'basemove_done':'STOP','basemove_failed':'failure'})
        smach.StateMachine.add('ARMMOVE', Armmove(), 
                               transitions={'armmove_done':'HOLD','armmove_failed':'failure'})
        smach.StateMachine.add('HOLD', Hold(), 
                               transitions={'hold_done':'HOLD','hold_failed':'ARMMOVE', 
                               'yes_adjustment':'ADJUST', 'job_done':'success'})
        smach.StateMachine.add('ADJUST', Adjust(), 
                               transitions={'being_adjusted':'ADJUST', 'start_over':'STOP', 'no_adjustment':'HOLD'})

 
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
	main()
