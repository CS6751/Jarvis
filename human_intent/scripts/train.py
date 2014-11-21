#!/usr/bin/env python
import cybrain as cb
import numpy as np
from time import time

#READ DATA FROM DATA FILES
X = []
Y = []
file = open('traindata.txt')
for line in file:
	data = [float(d) for d in line.split(',')]
	X.append(data)
file.close()

file = open('trainout.txt')
for line in file:
	data = float(line)
	Y.append([data])
file.close()


#CONVERT DATA TO NUMPY ARRAY
X, Y = np.array(X), np.array(Y)

#CREATE NETWORK
nnet = cb.Network()

#CREATE LAYERS
Lin = cb.Layer( 2, names= ['a','b'] )
Lhidden = cb.Layer( 2, cb.LogisticNeuron , names= ['c','d'] )
Lout = cb.Layer( 1, cb.LogisticNeuron , names= ['e'] )
bias = cb.Layer( 1, cb.BiasUnit, names= ['bias'] )

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
batch = cb.Trainer( nnet, X, Y, rate )

#TRAIN
t1 = time()
batch.epochs(10000)
print "Time CyBrain {}".format(time()-t1)

#PRINT RESULTS
for x in X:
    print "{} ==> {}".format( x, nnet.activateWith(x, return_value= True) )