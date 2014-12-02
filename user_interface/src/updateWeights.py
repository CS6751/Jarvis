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

def updateWeights(grips, utterance, axisAlignedBox, person_rot, plotting = True):

    # get the position and x-y dimensions in the box frame 
    # Assume: 
    #   1) the box is thin, with l,w representing the 'largest' components
    #   2) only one box exists at a time
    xspan = axisAlignedBox.w
    yspan = axisAlignedBox.l
    position = axisAlignedBox.pose.position # position in camera frame
    orientation = axisAlignedBox.pose.orientation # orientation wrt camera frame (Q: should this come in as a tf?)

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
    elif utterance.find("bottom left") > -1:
        angleInPersonFrame = -0.75*pi
    elif utterance.find("bottom right") > -1:
        angleInPersonFrame = -0.25*pi
    elif utterance.find("top left") > -1:
        angleInPersonFrame = 0.75*pi
    elif utterance.find("top right") > -1:
        angleInPersonFrame = 0.25*pi
    elif utterance.find("left") > -1:
        angleInPersonFrame = pi
    elif utterance.find("right") > -1:
        angleInPersonFrame = 0
    elif utterance.find("bottom") > -1:
        angleInPersonFrame = -0.5*pi
    elif utterance.find("top") > -1:
        angleInPersonFrame = 0.5*pi
    else:
        print 'Error: I thought I heard a location keyword, but I did not understand it.'

    if angleInPersonFrame:
        # Transform from the person frame into the box frame and then compute the likelihoods
        if not person_rot:
            # No person in view: default to using camera frame
            person_rot = 0

        c = cos(angleInPersonFrame)
        s = sin(angleInPersonFrame)

        likelihood['BoxLeft']       = max(-c, 0)**2
        likelihood['BoxTopLeft']    = max(-c, 0)**2 + max(s, 0)**2
        likelihood['BoxTop']        = max(s, 0)**2
        likelihood['BoxTopRight']   = max(c, 0)**2 + max(s, 0)**2
        likelihood['BoxRight']      = max(c, 0)**2
        likelihood['BoxBottomRight']= max(c, 0)**2 + max(-s, 0)**2
        likelihood['BoxBottom']     = max(-s, 0)**2
        likelihood['BoxBottomLeft'] = max(-c, 0)**2 + max(-s, 0)**2

    for i in range(len(oldGrips)):
        position = oldGrips[i].point
        zSum = 0
        for name, like in enumerate(likelihood):
            zSum += likelihood[name] * var[name].pdf(position)

        # Update the ith grip's weight.  posterior = grip weight * prior
        weight[i] = zSum * oldGrips[i].weight
        grip[i] = GraspBox(oldGrips[i].point, oldGrips[i].orientation, weight[i])

        newgrips.grasps.append(grip[i])

    if plotting:
        y, x = mgrid[slice(-0.5*yspan, 0.5*yspan + 0.1, 0.1),
                slice(-0.5*xspan, 0.5*xspan + 0.1, 0.1)]
        position = empty(x.shape + (2,))
        position[:,:,0] = x; position[:,:,1] = y
        z = var.pdf(position)
        plt.pcolor(x, y, z, cmap='spectral')
        for i in range(len(oldGrips)):
            #TODO: plot the points
            pass
        plt.show()

    newgrips.header.frame_id = '1'

    return newgrips

if __name__ == "__main__":
     updateWeights()
