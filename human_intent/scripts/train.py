#!/usr/bin/env python
import cybrain as cb
import numpy as np
from time import time

#READ DATA FROM DATA FILES
X = []
Y = []

file = open('traindata.txt')
for line in file:
	data = []
	for d in line.split(','):
		if d != '\n':
			data.append(float(d))
	# data = [float(d) for d in line.split(',')]
	X.append(data)
file.close()

file = open('trainout.txt')
for line in file:
	data = []
	for d in line.split(','):
		if d != '\n':
			data.append(float(d))
	Y.append(data)
file.close()

correct = 0
for i in range(len(Y)):
	trainX = []
	trainY = []
	testX = []
	testY = []
	for j in range(len(Y)):
		if j==i:
			testX.append(X[j])
			testY.append(Y[i])
		else:
			trainX.append(X[j])
			trainY.append(Y[j])

	# print trainX
	# print trainY
	# print testX
	# print testY
	#CONVERT DATA TO NUMPY ARRAY
	trainX, trainY = np.array(trainX), np.array(trainY)
	testX = np.array(testX)

	#CREATE NETWORK
	nnet = cb.Network()

	#CREATE LAYERS
	Lin = cb.Layer(12)
	Lhidden = cb.Layer( 5, cb.LogisticNeuron)
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
	t1 = time()
	batch.epochs(15)
	print "Time CyBrain {}".format(time()-t1)

	result = nnet.activateWith(testX[0], return_value= True)
	#PRINT RESULTS
	# for x in testX:
	#     print "{} ==> {}".format( x, result )
	print testY[0]
	print result[0]
	predict = result[0]

