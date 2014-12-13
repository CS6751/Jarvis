#!/usr/bin/env python

import rospy
import jarvis_perception
from jarvis_perception import findGrips
from jarvis_perception.srv import *
import geometry_msgs
from jarvis_perception.msg import GraspArray

class GraspGen:
    'listens to grasp topic and handles service requests for grasps'
    #variables to keep track of 
    hand_pos = None
    last_grasp = None
    test_pose = None
    test_weight = [0.5]

    def __init__(self, fake_grips):
        #generate a test pose to turn into a grasp
        header = std_msgs.msg.Header()
        header.frame_id = "board_tf"
        pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0,0,0),
                geometry_msgs.msg.Quaternion(0,0,0,1))
        self.test_pose = [geometry_msgs.msg.PoseStamped(header,pose)]

# use different callbacks in case we need to revert to fake grips
        if fake_grips:
            rospy.loginfo('using fake grips')
            serv = rospy.Service('return_grips', 
                    jarvis_perception.srv.ReturnGrips, 
                    self.handle_return_fake_grips)
        else:
            serv = rospy.Service('return_grips', 
                    jarvis_perception.srv.ReturnGrips, 
                    self.handle_return_grips)
            rospy.loginfo('using real grips')
        
        rospy.loginfo('Started Grip Server')
        
    # subscribe to grip topic 
    #    sub_ = rospy.Subscriber("grip_pt",
    #            geometry_msgs.PoseStamped,
    #            self.logGrasp)
        
        rospy.spin()
    
    def handle_return_fake_grips(self,req):
        grips = findGrips.findGrips()
        return jarvis_perception.srv.ReturnGripsResponse(grips)
    
    def handle_return_grips(self,req):
        grips = self.genGraspArray(self.test_pose,self.test_weight)
        return jarvis_perception.srv.ReturnGripsResponse(grips)
    
    def logGrasp(self,data):
        self.last_grasp = data

    def genGraspFromPose(self,stamped_pose, weight):
        #generates a weighted jarvis grasp from a pose
        grasp = jarvis_perception.msg.GraspBox()
        grasp.point = stamped_pose.pose.position
        grasp.orientation = stamped_pose.pose.orientation
        grasp.weight = weight
        return grasp

    def genGraspArray(self,poses, weights):
        #generates an array of grasps from an array of stamped poses
        #note: assumes that all poses have the same frame 
        #note: assumes weights and poses are the same length
        grasp_array = jarvis_perception.msg.GraspArray()
        for i in range(0,len(poses)):
            grasp_array.header.frame_id = poses[i].header.frame_id
            grasp_array.grasps.append(self.genGraspFromPose(poses[i],weights[i]))
        return grasp_array

if __name__ == "__main__":
    
    rospy.init_node('return_grips_server')
    fake_grips = True
    fake_grips = rospy.get_param("fake_grips",True)
    g =GraspGen(fake_grips)

