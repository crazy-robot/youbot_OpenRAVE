from openravepy import *
import time
from matplotlib.pyplot import *
from scipy import*

env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
env.Load('../environment/youbot_move.env.xml') # load a scene
robot = env.GetRobots()[0] # get the first robot
robot.SetActiveManipulator('arm')
goal = [0,1.5,1.5,-2,0]
# normal planning
transparency = 0.8
manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
starttime = time.time()
traj = manipprob.MoveManipulator(goal=goal,execute=False,outputtrajobj=True)
spec=traj.GetConfigurationSpecification() # get the configuration specification of the trajrectory
with env:
	T=robot.GetTransform()
	robotClone = env.ReadRobotURI(robot.GetURI())
	robotClone.SetName(robotClone.GetName()+u'clone')
	for link in robotClone.GetLinks():
		for geom in link.GetGeometries():
  			geom.SetTransparency(transparency)
	env.Add(robotClone,True)
robotClone.SetTransform(T)
env.UpdatePublishedBodies()

with env:
	T=robot.GetTransform()
	robotClone1 = env.ReadRobotURI(robot.GetURI())
	robotClone1.SetName(robotClone1.GetName()+u'clone1')
	for link in robotClone1.GetLinks():
		for geom in link.GetGeometries():
  			geom.SetTransparency(transparency)
	env.Add(robotClone1,True)
robotClone1.SetTransform(T)
env.UpdatePublishedBodies()

with env:
	T=robot.GetTransform()
	robotClone2 = env.ReadRobotURI(robot.GetURI())
	robotClone2.SetName(robotClone2.GetName()+u'clone2')
	for link in robotClone2.GetLinks():
		for geom in link.GetGeometries():
  			geom.SetTransparency(transparency)
	env.Add(robotClone2,True)
robotClone2.SetTransform(T)
env.UpdatePublishedBodies()

with env:
	T=robot.GetTransform()
	robotClone3 = env.ReadRobotURI(robot.GetURI())
	robotClone3.SetName(robotClone3.GetName()+u'clone3')
	for link in robotClone1.GetLinks():
		for geom in link.GetGeometries():
  			geom.SetTransparency(transparency)
	env.Add(robotClone3,True)
robotClone3.SetTransform(T)
env.UpdatePublishedBodies()

for i in range(2):
    starttime = time.time()
    while time.time()-starttime < 0.2:
	print traj.GetDuration()
        curtime = time.time()-starttime
        with env: # have to lock environment since accessing robot
            trajdata=traj.Sample(curtime)
            values=spec.ExtractJointValues(trajdata,robot,range(robot.GetDOF()),0)
	    print values
	    robotClone.SetDOFValues(values)
	
	time.sleep(0.01)

print "First trajectory finished"


for i in range(2):
    starttime = time.time()
    while time.time()-starttime < 0.4:
	print traj.GetDuration()
        curtime = time.time()-starttime
        with env: # have to lock environment since accessing robot
            trajdata=traj.Sample(curtime)
            values=spec.ExtractJointValues(trajdata,robot,range(robot.GetDOF()),0)
	    print values
	    robotClone1.SetDOFValues(values)
	
	time.sleep(0.01)

for i in range(2):
    starttime = time.time()
    while time.time()-starttime < 0.6:
	print traj.GetDuration()
        curtime = time.time()-starttime
        with env: # have to lock environment since accessing robot
            trajdata=traj.Sample(curtime)
            values=spec.ExtractJointValues(trajdata,robot,range(robot.GetDOF()),0)
	    print values
	    robotClone2.SetDOFValues(values)
	
	time.sleep(0.01)

for i in range(2):
    starttime = time.time()
    while time.time()-starttime < traj.GetDuration():
	print traj.GetDuration()
        curtime = time.time()-starttime
        with env: # have to lock environment since accessing robot
            trajdata=traj.Sample(curtime)
            values=spec.ExtractJointValues(trajdata,robot,range(robot.GetDOF()),0)
	    print values
	    robotClone3.SetDOFValues(values)
	
	time.sleep(0.01)


while True:
	a = 1
