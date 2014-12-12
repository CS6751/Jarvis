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

        plotting = True

        self.likelihood = None
        self.var = None

        try:
            grip_server = rospy.ServiceProxy('return_grips',ReturnGrips)
            self.grips = grip_server()
            pubgrips = rospy.Publisher('return_grips', jarvis_perception.msg.GraspArray, queue_size=10)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        listener = tf.TransformListener()

        #Initialize the plot
        plt.ion()
        fig = plt.figure()
        plthdl, = plt.plot([],[])

        pubgoal = rospy.Publisher('robot_cmd', GoalID)
        rospy.Subscriber('recognizer/output', String, self.speechCb)
        rospy.Subscriber('objects/target', jarvis_perception.msg.AxisAlignedBox, self.targetObjectCb)

        #now, update the weights and publish the result
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():

            try:
                (boardToPersonTranslationAll,boardToPersonRotationAll) = listener.lookupTransform('/board_tf', '/helmet', rospy.Time(0))
                self.boardToPersonRotation = boardToPersonRotationAll[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            # rospy.loginfo(gripsWithUpdatedWeights)
            print self.lastUtterance
            # print self.axisAlignedBox
            #rospy.loginfo(self.msg)

            if plotting and not self.likelihood == None:
                xspan = self.axisAlignedBox.w
                yspan = self.axisAlignedBox.l
                oldGrips = self.grips.grips.grasps
                plt.clf
                y, x = mgrid[slice(-0.5*yspan, 0.5*yspan + 0.005, 0.005),
                        slice(-0.5*xspan, 0.5*xspan + 0.005, 0.005)]
                posArray = empty(x.shape + (2,))
                posArray[:,:,0] = x; posArray[:,:,1] = y
                z = 0
                for j, name in enumerate(self.likelihood):
                    z += self.likelihood[name] * self.var[name].pdf(posArray)
                plt.pcolor(x, y, z, cmap='spectral')
                for i in range(len(oldGrips)):
                    plt.plot([0, self.newGrips.grasps[i].point.x], [0, self.newGrips.grasps[i].point.y], linestyle='None', marker='o', markerfacecolor='blue', markersize=12)
                plt.draw()
                plt.clf()

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
                self.lastUtterance = 'right'
                self.boardToPersonRotation = random.rand() #0.0
                self.grips.grips.grasps[0].point.x = 0.1*random.rand()
                self.newGrips, self.likelihood, self.var = updateWeights.updateWeights(self.grips, self.lastUtterance, self.axisAlignedBox, self.boardToPersonRotation)
            else:
                self.newGrips = self.grips

    def targetObjectCb(self, msg):
        #rospy.loginfo(msg)
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
