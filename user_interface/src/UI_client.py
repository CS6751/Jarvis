#! /usr/bin/env python

import sys
import rospy
from jarvis_perception import *
import geometry_msgs
from jarvis_perception.srv import *
import jarvis_perception.msg
import updateWeights
import roslib; roslib.load_manifest('pocketsphinx')
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
import tf
from numpy import *
import matplotlib.pyplot as plt

class UI_client:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        rospy.wait_for_service('return_grips')
        self.msg = GoalID()
        self.newGrips = jarvis_perception.msg.GraspArray()

        self.lastUtterance = ''
        self.axisAlignedBox = ''
        self.person_trans = ''
        self.person_rot = ''

        try:
            grip_server = rospy.ServiceProxy('return_grips',ReturnGrips)
            self.grips = grip_server()
            pubgrips = rospy.Publisher('return_grips', jarvis_perception.msg.GraspArray, queue_size=10)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # listener = tf.TransformListener()
        # try:
        #     (self.person_trans,self.person_rot) = listener.lookupTransform('/person_tf', '/<box tf>', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue

        ## TODO: implement listener for person pose topic

        # rospy.wait_for_service('spawn')
        # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        # spawner(4, 2, 0, 'turtle2')

        #Initialize the plot
        # plt.ion()
        # self.fig = plt.figure()
        # plt.show()
        # self.plthdl, = plt.plot([],[])
        # plt.draw()

        pubgoal = rospy.Publisher('robot_cmd', GoalID)
        rospy.Subscriber('recognizer/output', String, self.speechCb)
        rospy.Subscriber('objects/target', jarvis_perception.msg.AxisAlignedBox, self.targetObjectCb)

        #now, update the weights and publish the result
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            # rospy.loginfo(gripsWithUpdatedWeights)
            print self.lastUtterance
            # print self.axisAlignedBox
            rospy.loginfo(self.msg)

            pubgoal.publish(self.msg)
            pubgrips.publish(self.newGrips)
            r.sleep()

    def speechCb(self, msg):
        rospy.loginfo(msg.data)
        self.lastUtterance = msg.data
        # print "Hello!!!"

        # a command keyword is found -- update the goalid
        # 1. come here
        # 2. grab this
        # 3. stop!

        if msg.data.find("come here") > -1:
            self.msg.id = 'come_here'

        elif msg.data.find("grab") > -1:
            self.msg.id = 'grab'

        elif msg.data.find("stop") > -1:
            self.msg.id = 'stop'

        # a locative keyword is found -- update the probabilities/weights
        else:
            if self.axisAlignedBox:
                # self.lastUtterance = 'right'
                self.person_rot = random.rand() #0.0
                self.grips.grips.grasps[0].point.x = 0.1*random.rand()
                self.newGrips = updateWeights.updateWeights(self.grips, self.lastUtterance, self.axisAlignedBox, self.person_rot, True)
            else:
                self.newGrips = self.grips

    def targetObjectCb(self, msg):
        rospy.loginfo(msg)
        self.axisAlignedBox = msg

    # def return_grips_client():

    #     try:
    #         grip_server = rospy.ServiceProxy('return_grips',ReturnGrips)
    #         grips = grip_server()
    #     except rospy.ServiceException, e:
    #         print "Service call failed: %s"%e

    #     #now, update the weights and publish the result
    #     try:
    #         pub = rospy.Publisher('return_grips', jarvis_perception.msg.GraspArray, queue_size=10)
    #         r = rospy.Rate(10) # 10hz
    #         while not rospy.is_shutdown():
    #             command = 3
    #             rospy.loginfo(command)
    #             pub.publish(rospy.get_time(),command)

    #             gripsWithUpdatedWeights = updateWeights.updateWeights(grips)
    #             rospy.loginfo(gripsWithUpdatedWeights)
    #             pub.publish(gripsWithUpdatedWeights)
    #             print "Publishing..."
    #             r.sleep()
    #     except rospy.ROSInterruptException: pass 

    def usage(self):
        return "requests grip positions/weights from the perception module"

    def cleanup(self):
        # stop the robot!
        command = GoalID()
        rospy.loginfo(command)
        self.pubgoal.publish(command)

if __name__ == "__main__":
    rospy.init_node('UI_client')
    try:
        UI_client()
    except:
        pass
