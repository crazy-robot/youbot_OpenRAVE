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
joint1 = []
joint2 = []
joint3 = []
joint4 = []
joint5 = []
joint11 = []
joint22 = []
joint33 = []
joint44 = []
joint55 = []
joint111 = []
joint222= []
joint333 = []
joint444 = []
joint555 = []
point = 0
points=0
# normal planning
manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
starttime = time.time()
traj = manipprob.MoveManipulator(goal=goal,execute=False,outputtrajobj=True)
spec=traj.GetConfigurationSpecification() # get the configuration specification of the trajrectory
for i in range(2):
    starttime = time.time()
    while time.time()-starttime < traj.GetDuration():
        curtime = time.time()-starttime
        with env: # have to lock environment since accessing robot
            trajdata=traj.Sample(curtime)
            values=spec.ExtractJointValues(trajdata,robot,range(robot.GetDOF()),0)
            if points <1 :
		value = values/0.01
		#val = value/0.001
	    else:
		value = (values-valuesold)/0.01
		#val = (value-valueold)/0.001
		
	    joint1.append(values[2])
	    joint2.append(values[3])
            joint3.append(values[4])
            joint4.append(values[5])
            joint5.append(values[6])
	    joint11.append(value[2])
	    joint22.append(value[3])
            joint33.append(value[4])
            joint44.append(value[5])
            joint55.append(value[6])
	    
	    points+=1
            robot.SetDOFValues(values)
	    valuesold = values
	    valueold = value
        time.sleep(0.01)


point = points
fig=figure()
#fig.xlabel('Time Sample resolution of 0.01s')
#fig.ylabel('Position of each joints1-5')
ax = fig.add_subplot(111)
ax.set_xlabel('Sampled Time Resolution=0.01s')
ax.set_ylabel('Position at each sampled time in radians')
a1 = fig.add_subplot(5,1,1)
a2 = fig.add_subplot(5,1,2)
a3 = fig.add_subplot(5,1,3)
a4 = fig.add_subplot(5,1,4)
a5 = fig.add_subplot(5,1,5)
y = linspace(1,points,points)
a1.plot(joint1)
a2.plot(joint2)
a3.plot(joint3)
a4.plot(joint4)
a5.plot(joint5)
fig1=figure()
ay = fig1.add_subplot(111)
ay.set_xlabel('Sampled Time Resolution=0.01s')
ay.set_ylabel('Velocity at each sampled time in radians/second')
a11 = fig1.add_subplot(5,1,1)
a22 = fig1.add_subplot(5,1,2)
a33 = fig1.add_subplot(5,1,3)
a44 = fig1.add_subplot(5,1,4)
a55 = fig1.add_subplot(5,1,5)
y = linspace(1,point,point)
a11.plot(y,joint11)
a22.plot(y,joint22)
a33.plot(y,joint33)
a44.plot(y,joint44)
a55.plot(y,joint55)
#ax.xlabel('Number of tests peformed')
#.ylabel('Time taken for correspoding test')
show()

