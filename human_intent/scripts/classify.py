#!/usr/bin/env python

def classify():
	print 'doing classification'

if __name__ == '__classify__':
    try:
        classify()
    except rospy.ROSInterruptException :pass
