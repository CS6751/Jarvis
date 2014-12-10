import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from human_intent.msg import Intent
from jarvis_planner.msg import PlanStatus
from jarvis_executive.msg import *

'''
def user_interface_callback(userdata):
    if userdata.id== '10':
      print 'correct!!!'
    else:
      print 'go away..'
def human_intent_callback(userdata):
    if userdata.intent == 1:
        print 'intent Hold!'
    else:
	    print 'intent Adjust!'
def jarvis_planner_callback(userdata):
    if userdata.PlanStatus == True:
    	print 'plan ready!'
    else:
    	print 'wait for planning...'
'''

num = 0
def callback(userdata):
    if userdata.kill:
        print 'listening Kill topic!!!'
        print num
    	
def listener():
    rospy.init_node('test_listening', anonymous=True)
    #rospy.Subscriber('robot_cmd_trial', GoalID, user_interface_callback)
    #rospy.Subscriber('intents_trial', Intent, human_intent_callback)
    #rospy.Subscriber('PlanStatus_trial', PlanStatus, jarvis_planner_callback)
    rospy.Subscriber('Kill', Kill, callback)
    num += 1
    rospy.spin()
    
if __name__ == '__main__':
    listener()