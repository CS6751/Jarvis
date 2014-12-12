#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from jarvis_perception.msg import GraspBox
from jarvis_perception.msg import GraspArray
import math
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
from numpy import *
from collections import OrderedDict

def updateWeights(grips, utterance, axisAlignedBox, person_rot):

    # get the position and x-y dimensions in the box frame 
    # Assume: 
    #   1) the box is thin, with l,w representing the 'largest' components
    #   2) only one box exists at a time
    xspan = axisAlignedBox.w
    yspan = axisAlignedBox.l
    position = axisAlignedBox.pose.position # position in camera frame
    orientation = axisAlignedBox.pose.orientation # orientation wrt camera frame (Q: should this come in as a tf?)
    print utterance
    # print xspan, yspan, position, orientation

    #TODO: is the origin at the center of the box? 
    var = OrderedDict()
    var['BoxAnywhere']      = multivariate_normal(mean=[0,0],                   cov=[[10*xspan,0],[0,10*yspan]])
    var['BoxCenter']        = multivariate_normal(mean=[0,0],                   cov=[[0.5*xspan,0],[0,0.5*yspan]])

    var['BoxLeft']          = multivariate_normal(mean=[-0.5*xspan,0],          cov=[[0.05*xspan,0],[0,0.3*yspan]])
    var['BoxRight']         = multivariate_normal(mean=[0.5*xspan,0],           cov=[[0.05*xspan,0],[0,0.3*yspan]])
    var['BoxBottom']        = multivariate_normal(mean=[0,-0.5*yspan],          cov=[[0.3*xspan,0],[0,0.05*yspan]])
    var['BoxTop']           = multivariate_normal(mean=[0,0.5*yspan],           cov=[[0.3*xspan,0],[0,0.05*yspan]])

    var['BoxBottomLeft']    = multivariate_normal(mean=[-0.5*xspan,-0.5*yspan], cov=[[0.05*xspan,0],[0,0.05*yspan]])
    var['BoxBottomRight']   = multivariate_normal(mean=[0.5*xspan,-0.5*yspan],  cov=[[0.05*xspan,0],[0,0.05*yspan]])
    var['BoxTopLeft']       = multivariate_normal(mean=[-0.5*xspan,0.5*yspan],  cov=[[0.05*xspan,0],[0,0.05*yspan]])
    var['BoxTopRight']      = multivariate_normal(mean=[0.5*xspan,0.5*yspan],   cov=[[0.05*xspan,0],[0,0.05*yspan]])


    oldGrips = grips.grips.grasps
    # print "%s"%oldGrips[0]

    newgrips = GraspArray()
    weight = {}
    grip = {}

    # Assign a likelihood based on the keyword and the person pose
    likelihood = OrderedDict([('BoxAnywhere',0),('BoxCenter',0),('BoxLeft',0),('BoxRight',0),('BoxBottom',0),('BoxTop',0),('BoxBottomLeft',0),('BoxBottomRight',0),('BoxTopLeft',0),('BoxTopRight',0)])
    angleInPersonFrame = None
    if utterance.find("anywhere") > -1:
        likelihood['BoxAnywhere'] = 1
    elif utterance.find("center") > -1:
        likelihood['BoxCenter'] = 1
    elif utterance.find("bottom left") > -1 or utterance.find("lower left") > -1:
        angleInPersonFrame = -0.75*pi
    elif utterance.find("bottom right") > -1 or utterance.find("lower right") > -1:
        angleInPersonFrame = -0.25*pi
    elif utterance.find("top left") > -1 or utterance.find("upper left") > -1:
        angleInPersonFrame = 0.75*pi
    elif utterance.find("top right") > -1 or utterance.find("upper right") > -1:
        angleInPersonFrame = 0.25*pi
    elif utterance.find("left") > -1:
        angleInPersonFrame = pi
    elif utterance.find("right") > -1:
        angleInPersonFrame = 1e-5
    elif utterance.find("bottom") > -1:
        angleInPersonFrame = -0.5*pi
    elif utterance.find("top") > -1:
        angleInPersonFrame = 0.5*pi
    else:
        raise RuntimeWarning('Error: I thought I heard a location keyword, but I did not understand it.')

    if not angleInPersonFrame == None:
        # Transform from the person frame into the box frame and then compute the likelihoods
        if not person_rot:
            # No person in view: default to using camera frame
            person_rot = 0

        c = cos(angleInPersonFrame+person_rot)
        s = sin(angleInPersonFrame+person_rot)

        #TODO: normalize sum to one
        likelihood['BoxLeft']       = max(-c, 0)**2
        likelihood['BoxTopLeft']    = max(-c, 0)**2 + max(s, 0)**2
        likelihood['BoxTop']        = max(s, 0)**2
        likelihood['BoxTopRight']   = max(c, 0)**2 + max(s, 0)**2
        likelihood['BoxRight']      = max(c, 0)**2
        likelihood['BoxBottomRight']= max(c, 0)**2 + max(-s, 0)**2
        likelihood['BoxBottom']     = max(-s, 0)**2
        likelihood['BoxBottomLeft'] = max(-c, 0)**2 + max(-s, 0)**2

        # print likelihood['BoxTopLeft']
        # print likelihood['BoxLeft']
        # print likelihood['BoxBottomLeft']

    y, x = mgrid[slice(-0.5*yspan, 0.5*yspan + 0.001, 0.001),
            slice(-0.5*xspan, 0.5*xspan + 0.001, 0.001)]
    posArray = empty(x.shape + (2,))
    posArray[:,:,0] = x; posArray[:,:,1] = y
    z = 0
    for j, name in enumerate(likelihood):
        z += likelihood[name] * var[name].pdf(posArray)
    zMax = z.max() # normalizing the distribution - TODO: this is only an estimate-- fix
    # print zMax

    for i in range(len(oldGrips)):
        # pos = oldGrips[i].point[0]
        # print oldGrips[i]
        pos = 2*[[]]
        pos[0] = float(oldGrips[i].point.x); pos[1] = float(oldGrips[i].point.y)
        zSum = 0
        for j, name in enumerate(likelihood):
            # print name
            # print likelihood[name]
            # print var[name].pdf(pos)
            zSum += likelihood[name] * var[name].pdf(pos)
        # print zSum

        # Update the ith grip's weight.  (posterior) = (grip likelihood) * (prior)
        weight[i] = zSum/zMax * oldGrips[i].weight
        grip[i] = GraspBox(oldGrips[i].point, oldGrips[i].orientation, weight[i])

        newgrips.grasps.append(grip[i])
        # print grip[i]

    # newgrips.header.frame_id = '1'

    return newgrips, likelihood, var

if __name__ == "__main__":
     updateWeights()
