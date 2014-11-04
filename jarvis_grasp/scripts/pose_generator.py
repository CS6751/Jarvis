#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from jarvis_grasp.msg import goal_pose
from jarvis_perception.msg import GraspArray, GraspBox

class Pose_Generator:
    def __init__(self):
        self.pub = rospy.Publisher('goal_pose', goal_pose, queue_size=10)
        rospy.Subscriber('grasp', GraspArray, self.callback)

    def callback(self, msg):
        msg = goal_pose()
        
        base_pose = Pose2D()
        base_pose.x = 0
        base_pose.y = 0
        base_pose.theta = 0
        msg.base_pose = base_pose
        
        msg.joint_angles = [0, 0, 0, 0, 0]
        
        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('pose_generator', anonymous=True)
        Pose_Generator()
        rospy.spin()
    except rospy.ROSInterruptException: pass
