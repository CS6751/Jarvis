#!/usr/bin/env python
# make prediction of human intent depending on the motor data

#import youbot_common
from sensor_msgs.msg import JointState
import rospy
import numpy

buffer = numpy.zeros((5,7))

def prediction():
    rospy.init_node('temp')
    sub = rospy.Subscriber('/joint_states',JointState,callback)
    print '1'

def callback(msg):
    print 'collected msg'
    rawdata = [d.strip for d in msg.split(',')]
    effort_data = rawdata[25:31]
    print effort_data


if __name__ == '__main__':
    try:
        prediction()
    except rospy.ROSInterruptException :pass



