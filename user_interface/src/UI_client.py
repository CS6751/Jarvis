#! /usr/bin/env python

import sys
import rospy
from jarvis_perception import *
import geometry_msgs
from jarvis_perception.srv import *
import jarvis_perception.msg
from visualization_msgs.msg import Marker
import updateWeights
import roslib; roslib.load_manifest('pocketsphinx')
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
import tf
from tf.transformations import euler_from_quaternion
from numpy import *
import matplotlib.pyplot as plt
from math import sin, cos, atan2

class UI_client:

    def __init__(self):
        """
        JARVIS User Interface Node
        """
        rospy.on_shutdown(self.cleanup)
        rospy.wait_for_service('return_grips')
        self.msg = GoalID()
        self.newGrips = jarvis_perception.msg.GraspArray()

        self.legalUtterances = ['upper','lower','top','bottom','back','front','near','far','rear','left','right','anywhere','center']
        self.lastUtterance = ''
        self.axisAlignedBox = ''
        self.person_trans = ''
        self.person_rot = ''

        plotting = True

        self.likelihood = None
        self.var = None

        try:
            grip_server = rospy.ServiceProxy('return_grips',ReturnGrips)
            self.grips = grip_server()
            print self.grips
            pubgrips = rospy.Publisher('return_grips', jarvis_perception.msg.GraspArray, queue_size=10)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        listener = tf.TransformListener()

        #Initialize the plot
        plt.ion()
        fig = plt.figure(figsize=(8, 10))
        plthdl, = plt.plot([],[])
        plt.axis('equal')

        pubgoal = rospy.Publisher('robot_cmd', GoalID)
        rospy.Subscriber('recognizer/output', String, self.speechCb)
        rospy.Subscriber('board_vis', Marker, self.targetObjectCb)

        #now, update the weights and publish the result
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (boardToPersonTranslationAll,boardToPersonRotationAll) = listener.lookupTransform('/board_tf', '/helmet', rospy.Time(0))
                self.boardToPersonRotationAll = euler_from_quaternion(boardToPersonRotationAll)
                self.boardToPersonRotationAll = self.alignAxes()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            # rospy.loginfo(gripsWithUpdatedWeights)
            print self.lastUtterance
            # print self.axisAlignedBox
            #rospy.loginfo(self.msg)

            if plotting and not self.likelihood == None:
                xspan = self.axisAlignedBox.scale.z  # TODO: reversed????
                yspan = self.axisAlignedBox.scale.y
                xcent = -self.axisAlignedBox.pose.position.y
                ycent = self.axisAlignedBox.pose.position.z

                oldGrips = self.grips.grips.grasps
                plt.clf
                y, x = mgrid[slice(ycent-0.5*yspan, ycent+0.5*yspan + 0.005, 0.005),
                             slice(xcent-0.5*xspan, xcent+0.5*xspan + 0.005, 0.005)]
                posArray = empty(x.shape + (2,))
                posArray[:,:,0] = x; posArray[:,:,1] = y
                z = 0
                for j, name in enumerate(self.likelihood):
                    z += self.likelihood[name] * self.var[name].pdf(posArray)
                plt.pcolor(x, y, z, cmap='spectral')
                plt.axis('equal')
                for i in range(len(oldGrips)):
                    xgrip = self.newGrips.grasps[i].point.z
                    ygrip = -self.newGrips.grasps[i].point.y
                    weight = self.newGrips.grasps[i].weight
                    plt.plot([xgrip, xgrip], [ygrip, ygrip], linestyle='None', marker='o', markerfacecolor='blue', markersize=36*weight)
                plt.draw()
                plt.clf()

            try:
                pubgoal.publish(self.msg)
                pubgrips.publish(self.newGrips)
            except:
                rospy.loginfo("publish failed")
            r.sleep()

    def alignAxes(self):
        self.boardToPersonRotation = self.boardToPersonRotationAll[0]  # roll
        self.boardToPersonAzimuth = self.boardToPersonRotationAll[2]  # yaw
        # self.boardToPersonAzimuth = atan2(sin(boardToPersonRotationAll[2]), sin(boardToPersonRotationAll[1]))  # az = atan2(sin(yaw), sin(pitch))
        self.boardToPersonElevation = self.boardToPersonRotationAll[1]  # pitch
        # print "Board rotation: ", self.boardToPersonRotationAll
        # print "   Roll (with x-axis perpendicular to board): ", self.boardToPersonRotation
        # print "   Azimuth: ", self.boardToPersonAzimuth
        # print "   Elevation: ", self.boardToPersonElevation

        boardToPersonRotationAll = self.boardToPersonRotationAll
        return boardToPersonRotationAll

    def speechCb(self, msg):
        rospy.loginfo(msg.data)
        self.lastUtterance = msg.data

        # a command keyword is found -- update the goalid
        # 1. come here
        # 2. grab this
        # 3. stop!

        # self.lastUtterance = 'right'
        # self.boardToPersonRotation = random.rand() #0.0
        # self.grips.grips.grasps[0].point.x = 0.1*random.rand()

        if msg.data.find("come here") > -1:
            self.msg.id = 'come_here'

        elif msg.data.find("grab") > -1:
            self.msg.id = 'grab'

        elif msg.data.find("stop") > -1:
            self.msg.id = 'stop'

        # a locative keyword is found -- update the probabilities/weights
        elif any([self.lastUtterance.find(self.legalUtterances[i]) > -1 for i in range(len(self.legalUtterances))]):
            if self.axisAlignedBox:
                print "board rotation relative to user: ", self.boardToPersonRotation
                self.newGrips, self.likelihood, self.var = updateWeights.updateWeights(self.grips, self.lastUtterance, self.axisAlignedBox, self.boardToPersonRotationAll)
            else:
                self.newGrips = self.grips
        else:
            # rospy.logwarn("I thought I heard a location keyword, but I did not understand it.")
            raise RuntimeWarning("I thought I heard a location keyword, but I did not understand it.")
            self.newGrips = self.grips

    def targetObjectCb(self, msg):
        # rospy.loginfo(msg)
        self.axisAlignedBox = msg

    def usage(self):
        return "requests grip positions/weights from the perception module"

    def cleanup(self):
        # stop the robot!
        command = GoalID()
        #rospy.loginfo(command)
        self.pubgoal.publish(command)

if __name__ == "__main__":
    rospy.init_node('UI_client')
    try:
        UI_client()
    except:
        pass
