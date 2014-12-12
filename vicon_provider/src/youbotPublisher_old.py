#! /usr/bin/env python

import roslib
import rospy
import tf
import sys, time
from numpy import *
import _pyvicon

class ViconPublisher:
    def __init__(self, host="10.0.0.102", port=800, x_VICON_name="KUKAyouBot2:main body <t-X>", y_VICON_name="KUKAyouBot2:main body <t-Y>", theta_VICON_name="KUKAyouBot2:main body <a-Z>", z_VICON_name="KUKAyoubot2:main body <t-Z>"):
   # def __init__(self, host="10.0.0.102", port=800, x_VICON_name="GPSReceiverHelmet-goodaxes:GPSReceiverHelmet01 <t-X>", y_VICON_name="GPSReceiverHelmet-goodaxes:GPSReceiverHelmet01 <t-Y>", theta_VICON_name="GPSReceiverHelmet-goodaxes:GPSReceiverHelmet01 <a-Z>"):
        
        """
        Pose handler for VICON system
        host (string): The ip address of VICON system (default="10.0.0.102")
        port (int): The port of VICON system (default=800)
        x_VICON_name (string): The name of the stream for x pose of the robot in VICON system (default="SubjectName:SegmentName <t-X>")
        y_VICON_name (string): The name of the stream for y pose of the robot in VICON system (default="SubjectName:SegmentName <t-Y>")
        theta_VICON_name (string): The name of the stream for orintation of the robot in VICON system (default="SubjectName:SegmentName <a-Z>")
        """

        br = tf.TransformBroadcaster()

        self.host = host
        self.port = port
        self.x = x_VICON_name
        self.y = y_VICON_name
        self.z = z_VICON_name
        self.theta = theta_VICON_name

        self.s = _pyvicon.ViconStreamer()
        self.s.connect(self.host,self.port)

        self.s.selectStreams(["Time", self.x, self.y, self.z, self.theta])

        self.s.startStreams()

        # Wait for first data to come in
        while self.s.getData() is None: pass

        while not rospy.is_shutdown():
            print self.s.getData()
            pose = self.getPose()
            br.sendTransform((pose[1],pose[2],pose[3]),
                    tf.transformations.quaternion_from_euler(0,0,pose[4]),
                    rospy.Time.now(),
                    "youbot",
                    "vicon");
# Is any sort of sleep command needed?
    def _stop(self):
        print "youbot pose handler quitting..."
        self.s.stopStreams()
        print "Terminated."

    def getPose(self, cached=False):

        (t, x, y, z, o) = self.s.getData()
        (t, x, y, z, o) = [t/100, x/1000, y/1000, z/1000, o]

        return array([t, x, y, z, o])

if __name__ == "__main__":
    rospy.init_node('youbotPublisher')
    try:
        ViconPublisher()
    except:
        pass
