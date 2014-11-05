#!/usr/bin/env python

import rospy
from jarvis_grasp.msg import goal_pose #Alex
from geometry_msgs.msg import Pose2D #robot state from ROS, need conf
from jarvis_executive.msg import PlanCommand, Kill #Elijah
from jarvis_planner.msg import Trajectories, PlanStatus
from jarvis_perception.msg import AxisAlignedBox
from sensor_msgs.msg import JointState


# perception/map data??

def posecallback(data):
    print 'Planner says: Got the Pose'

def commandcallback(data):
    print 'Planner says: Got a Plan Commad'

def objectscallback(data):
    print 'Planner says: Got the position of the human hand'

def pose2Dcallback(data):
    print 'Planner says: Got current base pose'

def jointscallback(data):
    print 'Planner says: Got current joint angle positions'

def killcallback(data):
    print 'Planner says: Got Kill Status'


def TrajectoryGenerator():
    rospy.init_node('TrajectoryGenerator',anonymous=True)
    print 'initializing planner node'    

    pub0 = rospy.Publisher('PlannedTrajectory',Trajectories, queue_size=10)
    pub1 = rospy.Publisher('PlanStatus',PlanStatus, queue_size=10)
    pub2 = rospy.Publisher('Kill',Kill, queue_size=10)

    rospy.Subscriber('Pose2D',Pose2D, pose2Dcallback)
    rospy.Subscriber('goal_pose', goal_pose, posecallback)
    rospy.Subscriber('PlanCommand',PlanCommand, commandcallback)
    rospy.Subscriber('objects',AxisAlignedBox, objectscallback)
    rospy.Subscriber('joint_states',JointState, jointscallback)
    rospy.Subscriber('Kill',Kill,killcallback)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg0 = Trajectories()
        msg0.ArmJointPositions=[0,0,0,0,0]
	msg0.ArmJointVelocities=[0,0,0,0,0]
	msg0.BasePositions=[[0],[.1]]
	msg0.BaseOrientations=[.5]
	msg0.BaseLinVelocities=[[0.2],[.1]]
	msg0.BaseRotVelocities=[0]
	msg0.time_from_start=.2
	    
        msg1 = PlanStatus()
        msg1.PlanStatus = False

        msg2 = Kill()
        msg2.kill = False

	pub0.publish(msg0)
        pub1.publish(msg1)
        pub2.publish(msg2)
        r.sleep

if __name__ == '__main__':
   try:
     rospy.init_node('TrajectoryGenerator', anonymous = True)
     TrajectoryGenerator()
     rospy.spin()
   except rospy.ROSInterruptException: pass


    
