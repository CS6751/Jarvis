#!/usr/bin/env python
# make prediction of human intent depending on the motor data

from sensor_msgs.msg import JointState
import rospy
import numpy as np
# from Intent.msg import intent
from std_msgs.msg import String
import matplotlib.pyplot as plt

buf = []
prev = []
curr = []
buflength = 15
threshold = 0.005
spark = 0
predict = 0
x = []
y1 = []
y2 = []
y3 = []
yp1 = []
yp2 = []
yp3 = []

def prediction():
    # x = [1,2,3,4]
    # y = [1,2,3,4]
    # plt.plot(x,y)
    # plt.show()

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

    plt.plot(x,y1,x,y2,x,y3,x,yp1(x),x,yp2(x),x,yp3(x))
    plt.show()

    rospy.spin()

def callback(msg):
    # print 'collected msg'
    global buf, prev, curr, spark
    if msg.name[0]=='arm_joint_1':
        data = msg.position
        curr = data[1:4]
        # print curr
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
    global x, y1, y2, y3
    y1 = []
    y2 = []
    y3 = []
    x = range(len(buf))
    for line in buf:
        y1.append(line[0])
        y2.append(line[1])
        y3.append(line[2])
    print 'doing classification'
    features = extractfeatures(buf)
    print 'features: ', features


def extractfeatures(buf):
    global yp1, yp2, yp3, x
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
    features.append(abs(link1[0]-link1[len(link1)-1]))
    features.append(abs(link2[0]-link2[len(link2)-1]))
    features.append(abs(link3[0]-link3[len(link3)-1]))
    z1 = np.polyfit(x,link1,2)
    z2 = np.polyfit(x,link2,2)
    z3 = np.polyfit(x,link3,2)
    abslink1 = [abs(z1[i]) for i in range(len(z1)-1)]
    abslink2 = [abs(z2[i]) for i in range(len(z2)-1)]
    abslink3 = [abs(z3[i]) for i in range(len(z3)-1)]
    for a in abslink1:
        features.append(a)
    for a in abslink2:
        features.append(a)
    for a in abslink3:
        features.append(a)
    yp1 = np.poly1d(z1)
    yp2 = np.poly1d(z2)
    yp3 = np.poly1d(z3)
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



