#!/usr/bin/env python
# collect data using different options
# saves the data to a file which will be used later to classify

import rospy
import numpy

option = 0

def main():
	option = input("1:move 2:take 3:stop")
    buf = []
    prev = []
    curr = []
    buflength = 5
    threshold = 10
    spark = 0
    while(1):
    	data = input()
    	data = [float(d) for d in data.split(',')]
    	curr = data
        if spark==0:
            dif = diff(curr,prev)
            if max(dif) > threshold:
                buf.append(prev)
                buf.append(curr)
                spark = 1
        else:
            buf.append(curr)
            if len(buf) == buflength:
                writetodata()
                buf = []
                spark = 0
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

def writetodata(buf):
	file = open('traindata.txt')
	data = processdata(buf)
	file.write(data)
	file.write("\n")
	fils.close()

	file = open('trainout.txt')
	file.write(option)
	file.write("\n")
	file.close()

def processdata(buf):
	#TODO 
	return "aa"

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException :pass

