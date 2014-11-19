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

class UI_client:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        rospy.wait_for_service('return_grips')
        self.msg = GoalID()

        try:
            grip_server = rospy.ServiceProxy('return_grips',ReturnGrips)
            grips = grip_server()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        self.pubgoal = rospy.Publisher('goalID', GoalID)
        rospy.Subscriber('recognizer/output', String, self.speechCb)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            gripsWithUpdatedWeights = updateWeights.updateWeights(grips)
            rospy.loginfo(gripsWithUpdatedWeights)
            rospy.loginfo(self.msg)
            self.pubgoal.publish(self.msg)
            r.sleep()
            # testing

    def speechCb(self, msg):
        rospy.loginfo(msg.data)
        rospy.loginfo(msg)

        if msg.data.find("come here") > -1:
            self.msg.id = 1
        if msg.data.find("move back") > -1:
            self.msg.id = 2

        rospy.loginfo(self.msg)
        self.pubgoal.publish(self.msg)

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
