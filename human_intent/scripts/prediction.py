#!/usr/bin/env python
# make prediction of human intent depending on the motor data

from sensor_msgs.msg import JointState
import rospy
import numpy as np
import cybrain as cb
from std_msgs.msg import String
#import matplotlib.pyplot as plt

buf = []
prev = []
curr = []
buflength = 15
threshold = 0.003
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

def prediction():

    print 'train network....'
    trainNetwork()
    print 'trained!'

    rospy.init_node('temp')
    print 'starting subscriber'
    sub = rospy.Subscriber('joint_states',JointState,callback)

    print 'publishing' 
    pub = rospy.Publisher('intents', String, queue_size=10)
    # pub = rospy.Publisher('intents', String)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        inte = str(predict)
        pub.publish(inte)
        r.sleep()

    rospy.spin()

def callback(msg):
    global buf, prev, curr, spark, predict, classified
    if msg.name[0]=='arm_joint_1':
        data = msg.position
        curr = data[1:4]
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
                    classify(buf)
                    classified = 1
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
    global predict, nnet

    print 'doing classification'
    testX = np.array(extractfeatures(buf))
  
    result = nnet.activateWith(testX, return_value= True)
    pred = result[0]
    pred = pred[0]
    print result
    if float(str(pred)) > 0.5 :
        predict = 1
    else:
        predict = 2
    print predict


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

def trainNetwork():
    global nnet
    trainX = []
    trainY = []

    file = open('testdata.txt')
    for line in file:
        data = []
        for d in line.split(','):
            if d != '\n':
                data.append(float(d))
        trainX.append(data)
    file.close()

    file = open('testout.txt')
    for line in file:
        data = []
        for d in line.split(','):
            if d != '\n':
                data.append(float(d))
        trainY.append(data)
    file.close()

    trainX, trainY = np.array(trainX), np.array(trainY)

    #CREATE NETWORK
    nnet = cb.Network()

    #CREATE LAYERS
    Lin = cb.Layer(12)
    Lhidden = cb.Layer( 10, cb.LogisticNeuron)
    Lout = cb.Layer( 1, cb.LogisticNeuron)
    bias = cb.Layer( 1, cb.BiasUnit)

    #ADD LAYERS TO NETWORK
    nnet.addInputLayer(Lin)
    nnet.addLayer(Lhidden)
    nnet.addOutputLayer(Lout)
    nnet.addAutoInputLayer(bias)

    #CONNECT LAYERS
    Lin.connectTo(Lhidden)
    Lhidden.connectTo(Lout)
    bias.connectTo(Lhidden)
    bias.connectTo(Lout)

    #CREATE BATCH TRAINER
    rate = 0.1
    batch = cb.Trainer( nnet, trainX, trainY, rate )

    #TRAIN
    # t1 = time()
    batch.epochs(100)

if __name__ == '__main__':
    try:
        prediction()
    except rospy.ROSInterruptException :pass



