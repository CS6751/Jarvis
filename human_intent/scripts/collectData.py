#!/usr/bin/env python
# make prediction of human intent depending on the motor data

from sensor_msgs.msg import JointState
import rospy
import numpy as np
import cybrain as cb
from std_msgs.msg import String
from std_msgs.msg import Float64
import copy
from sys import exit
#import matplotlib.pyplot as plt

buf = []
prev = []
curr = []
buflength = 150
threshold = 0.002
# three flags
spark = 0
moving = 0
classified = 0
# prediction made 0:stop state 1:adjust state 2:take away state
predict = 0
x = []
yp1 = []
yp2 = []
yp3 = []
nnet = ''
data = [1,0,0]
flag1 = 0
flag2 = 0
label = ''

def main():
    global label
    label = input("1:move 2:take 3:stop")
 	
    rospy.init_node('temp')
    print 'starting subscriber'
    sub = rospy.Subscriber('joint_states2',Float64,callback1)
    sub = rospy.Subscriber('joint_states3',Float64,callback2)
    sub = rospy.Subscriber('joint_states4',Float64,callback)

    print 'publishing' 
    # pub = rospy.Publisher('intents', String, queue_size=10)
    pub = rospy.Publisher('intents', String)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        inte = str(predict)
        pub.publish(inte)
        r.sleep()

    rospy.spin()

def callback1(msg):
    global data, flag1
    data[0] = msg.data
    flag1 = 1

def callback2(msg):
    global data, flag1, flag2
    if flag1==1:
        flag1 = 0
        flag2 = 1
        data[1] = msg.data

def callback(msg):
    global buf, prev, curr, spark, predict, classified, data, flag2
    # if msg.name[0]=='arm_joint_1':
    # data = msg.position
    if flag2==1:
        flag2 = 0
        data[2] = msg.data
        curr = copy.copy(data)
        dif = diff(curr,prev)
        # print 'classified: ',classified, 'diff: ', dif
        if max(dif) < threshold and classified==1:
            print 'low'
            classified = 0
            predict = 0
        if spark==0:
            if max(dif) > threshold:
                buf.append(prev)
                buf.append(curr)
                spark = 1
                predict = 0
        else:
            buf.append(curr)
            if len(buf) == buflength:
                if classified == 0:
                    writedata(buf)
                    classified = 1
                buf = []
                spark = 0
            total = totalrow(buf)
        prev = curr

def writedata(buf):
	print 'writing data'
	global label
	features = extractfeatures(buf)
	datafile = open('finaldata.txt','a')
	outfile = open('finalout.txt','a')

	for f in features:
		datafile.write(str(f))
		datafile.write(',')
	datafile.write('\n')

	outfile.write(str(label))
	outfile.write('\n')
	exit(0)

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


def extractfeatures(buf):
    global yp1, yp2, yp3, x
    x = range(len(buf))
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
        main()
    except rospy.ROSInterruptException :pass



