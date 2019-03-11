#!/usr/bin/env python
import roslib
import rospy
import math 
import time

from scienceCacheSystem import scienceLab 

class science_node:
    def __init__(self):
        self.lab = scienceLab('ScienceLab', port='ttyACM1',baud='115200',pwmLimit=100,motorsID=[128,129])

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            #print(self.mctl)
            #self.lab.createMachine()
            r.sleep()

if __name__=="__main__":
    rospy.init_node('ScienceLab')
    #try:
    science_node()
    #except:
    #    pass
