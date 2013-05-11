#!/usr/bin/env python
from openravepy import *
from numpy import *
import time
RaveInitialize()
#RaveLoadPlugin('ybplugin')
transparency = 0.7
robotClone = []

try:
    env=Environment()
    env.Load('../environment/door1.env.xml')
    robot = env.GetRobots()[0]
    env.SetViewer('qtcoin')
    time.sleep(10)
    manip = robot.SetActiveManipulator('arm')
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.TranslationDirection5D)
    T = robot.GetTransform()
    for i in range(1):
    	with env:
		T[0,3]+=0.3
    		robotClone.append(env.ReadRobotURI(robot.GetURI()))
    		robotClone[i].SetName(robotClone[i].GetName()+u'clone')
        	for link in robotClone[i].GetLinks():
      			for geom in link.GetGeometries():
          			geom.SetTransparency(transparency)
    		env.Add(robotClone[i],True)
        robotClone[i].SetTransform(T)
	env.UpdatePublishedBodies()
    while True:
	a = 1 
finally:
    RaveDestroy()
