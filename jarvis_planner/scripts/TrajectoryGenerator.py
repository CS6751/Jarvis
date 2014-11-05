#!/usr/bin/env python

import rospy
from jarvis_grasp.msg import goal_pose #Alex
from geometry_msgs.msg import Pose2D #robot state from ROS, need conf
from jarvis_executive.msg import PlanCommand #Elijah
from jarvis_planner.msg import Trajectories, PlanStatus
from jarvis_perception.msg import objects
from sensor_msgs.msg import JointState

# perception/map data??


def TrajectoryGenerator():
    rospy.init_node('TrajectoryGenerator',anonymous=True)
    self.pub = rospy.Publisher('PlannedTrajectory',Trajectories, queue_size=10)
    self.pub = rospy.Publisher('PlanStatus',PlanStatus, queue_size=10)
    self.pub = rospy.Publisher('Kill',Kill, queue_size=10)

    rospy.Subscriber('goal_pose', goal_pose, self.callback)
    rospy.Subscriber('PlanCommand',PlanCommand, self.callback)
    rospy.Subscriber('objects',AxisAlignedBox, self.callback)
    rospy.Subscriber('joint_states',JointState, self.callback)

    def callback(self,msg):
	msg = Trajectories()
		
	msg.ArmJointPositions=[0,0,0,0,0]
	msg.ArmJointVelocities=[0,0,0,0,0]
	msg.BasePositions=[[0],[.1]]
	msg.BaseOrientations=[.5]
	msg.BaseLinVelocities=[[0.2],[.1]]
	msg.BaseRotVelocities=[0]
	msg.time_from_start=.2
	    
	self.pub.publish(msg)

    def callback(self,msg):
	msg.PlanStatus()
	    
	msg.PlanStatus=False

	self.pub.publish(msg)

    def callback(self,msg):
        msg.Kill()
                 
        msg.Kill=False
	    


    if __name__ == '__main__':
       try:
         rospy.init_node('TrajectoryGenerator', anonymous = True)
         TrajectoryGenerator()
         rospy.spin()
       except rospy.ROSInterruptException: pass


    
