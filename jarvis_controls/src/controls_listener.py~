#!/usr/bin/env python


##### Imports ######
import rospy

#Kill and Mode
from jarvis_controls.msg import Kill #These will need to be changed probably.
from jarvis_controls.msg import Mode

#For trajectories:
from moveit_msgs.msg import RobotTrajectory
import geometry_msgs.msg

#For robot pose:
from sensor_msgs.msg import JointState


###### Callbacks ######
def trajcallback(data):
	print 'Controls: trajectory received!'
     #low level API calls here, passing the desired trajectory in.
   
def modecallback(data):
	print 'Controls: stiffness/mode received!'
     #low level API calls here, changing stiffness  
 
def killcallback(data):
	print 'Controls: robot kill status received!'
     #low level API calls here, stopping motion, keeping gravity comp. 

def posecallback(data):
	print 'Controls: robot pose read!'
     #get current robot pose


###### Initiate Listeners #####
def controls_listener():
    rospy.init_node('controls_listener') #Start the listener node.
    
#Kill message subscribe
    rospy.Subscriber('Kill',Kill,killcallback)	
#Mode message subscribe
    rospy.Subscriber('Mode',Mode,modecallback)	
#Trajectory message subscribe
    rospy.Subscriber('PlannedTrajectory',RobotTrajectory,trajcallback)	
#Pose message subscribe
    rospy.Subscriber('joint_states',JointStates,posecallback)

    rospy.spin()
        
if __name__ == '__main__':
    controls_listener()
