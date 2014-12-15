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
    last_grasps = []
    last_weights = []
    grasp_index = 0
    test_pose = None
    test_weight = [0.5]
    sub_ = None

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
            sub_ = rospy.Subscriber("grasps",geometry_msgs.msg.PoseStamped,self.logGrasp)
        
        rospy.loginfo('Started Grip Server')
        rospy.spin()
    
    def handle_return_fake_grips(self,req):
        grips = findGrips.findGrips()
        return jarvis_perception.srv.ReturnGripsResponse(grips)
    
    def handle_return_grips(self,req):
        
        grips = self.genGraspArray(self.last_grasps,self.last_weights)
        return jarvis_perception.srv.ReturnGripsResponse(grips)
    
    def logGrasp(self,data):
        print "logging grasp"
        print "number of logged grasps"
        print len(self.last_grasps)
        if (len(self.last_grasps) == 0):
            self.last_grasps = [data]
            self.last_weights = [0.5]
        elif (len(self.last_grasps) < 3):
            self.last_grasps.append(data)
            self.last_weights.append(0.5)
        else:
            self.last_grasps[self.grasp_index] = data 
            self.last_weights[self.grasp_index] = 0.5
            if (self.grasp_index < len(self.last_grasps)-2):
                self.grasp_index += 1
            else:
                self.grasp_index = 0


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
        print "length of poses"
        print len(poses)
        grasp_array = jarvis_perception.msg.GraspArray()
        if (len(poses) > 0):
            for i in range(0,len(poses)):
                grasp_array.header.frame_id = poses[i].header.frame_id
                grasp_array.grasps.append(self.genGraspFromPose(poses[i],weights[i]))
        return grasp_array

if __name__ == "__main__":
    
    rospy.init_node('return_grips_server')
    fake_grips = False
    fake_grips = rospy.get_param("fake_grips",True)
    g =GraspGen(fake_grips)

