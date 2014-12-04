#!/usr/bin/env python
# make prediction of human intent depending on the motor data

from sensor_msgs.msg import JointState
import rospy
import numpy
# from Intent.msg import intent
from std_msgs.msg import String

buf = []
prev = []
curr = []
buflength = 15
threshold = 0.005
spark = 0
predict = 0

def prediction():

    rospy.init_node('temp')
    print 'starting'
    sub = rospy.Subscriber('joint_states',JointState,callback)

    print 'publish' 
    pub = rospy.Publisher('intents', String, queue_size=10)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        inte = str(predict)
        pub.publish(inte)
        r.sleep()

    rospy.spin()

def callback(msg):
    # print 'collected msg'
    global buf, prev, curr, spark
    if msg.name[0]=='arm_joint_1':
        data = msg.position
        curr = data[1:4]
        print curr
        if spark==0:
            dif = diff(curr,prev)
            # print "diff: ",dif
            if max(dif) > threshold:
                buf.append(prev)
                buf.append(curr)
                spark = 1
        else:
            buf.append(curr)
            if len(buf) == buflength:
                classify(buf)
                buf = []
                spark = 0
            total = totalrow(buf)
        prev = curr

def diff(v1, v2):
    if not v1 or not v2:
        return [0.0]*(len(v1)+len(v2))
    d = []
    for i in range(len(v1)):
        d.append(abs(v1[i]-v2[i]))
    return d

def totalrow(mat):
    if not mat:
        return []
    total = [0.0]*len(mat[0])
    for row in mat:
        for i in range(len(row)):
            total[i] = total[i] + row[i]
    return total

def classify(buf):
    print 'doing classification'
    features = extractfeatures(buf)
    print 'features: ', features


def extractfeatures(buf):
    features = []
    link1 = []
    link2 = []
    link3 = []
    for p in buf:
        link1.append(p[0])
        link2.append(p[1])
        link3.append(p[2])
    features.append(avgdelta(link1))
    features.append(avgdelta(link2))
    features.append(avgdelta(link3))
    return features

def avgdelta(l):
    maximum = max(l)
    minimum = min(l)
    maxindex = l.index(maximum)
    minindex = l.index(minimum)
    if maximum==minimum:
        return 0
    return abs(maximum-minimum)/abs(maxindex-minindex)


if __name__ == '__main__':
    try:
        prediction()
    except rospy.ROSInterruptException :pass



