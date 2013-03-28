
import h5py
import os
#rm -rf mydataset.hdf5
datafile = h5py.File('inversekinematic.hdf5')
current_robot = datafile.create_dataset("current",(1000,1,5),'f')
forward_current_robot = datafile.create_dataset("forward",(1000,4,4),'f')
result_robot = datafile.create_dataset("result",(1000,8,1,5),'f')
forward_result_robot = datafile.create_dataset("resultforward",(1000,8,4,4),'f')

from openravepy import *
from numpy import *
import time
from openravepy.misc import InitOpenRAVELogging

InitOpenRAVELogging() 
env = Environment() # create the environment
#env.SetViewer('qtcoin') # start the viewer
env.Load('../robots/youbot_5D.robot.xml')
robot = env.GetRobots()[0] # get the first robot

#current_robot = create_database('current_robot',(100,5,1),'f')
#current_robot = create_database('current_robot',(100,5,1),'f')
count_i = 0
count_j = 0

manip = robot.SetActiveManipulator('arm') # set the manipulator to leftarm
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.TranslationDirection5D)
if not ikmodel.load():
    ikmodel.autogenerate()

for check in range (0,1000):
    with env: # lock environment
        lower,upper = robot.GetDOFLimits(manip.GetArmIndices())
        a = lower+random.rand(len(lower))*(upper-lower)
        #print a
        robot.SetDOFValues(a,manip.GetArmIndices())
        #current_robot[check] =  array([array([a[0]]),array([a[1]]),array([a[2]]),array([a[3]]),array([a[4]])],dtype = float32)
        current_robot[check] = array(a,dtype=float32)
        ikparam = manip.GetIkParameterization(IkParameterization.Type.TranslationDirection5D)
        sols = manip.FindIKSolutions(ikparam,IkFilterOptions.CheckEnvCollisions)
        Tee = manip.GetEndEffectorTransform()
        forward_current_robot[check] = array(Tee,dtype=float32)
	if sols.shape[0] is 0:
		count_i = count_i +1
        for i in range(0,sols.shape[0]):
            with robot:
                a = sols[i]
                result_robot[check,i] = array(a,dtype=float32)
                robot.SetDOFValues(sols[i],manip.GetArmIndices()) # set the current solution
    	    	Tee = manip.GetEndEffectorTransform()
                forward_result_robot[check] = array(Tee,dtype=float32)
    	    	env.UpdatePublishedBodies() # allow viewer to update new robot
                #raveLogInfo('Tee is: '+repr(Tee)) 

print count_i
env.SetViewer('qtcoin') # start the viewer
time.sleep(10)
for j in range(0,1000):
   with env:
            with robot:
		a = result_robot[j,1]
                robot.SetDOFValues(a[0],manip.GetArmIndices()) # set the current solution
                env.UpdatePublishedBodies() # allow viewer to update new robot
		Tee = manip.GetEndEffectorTransform()
		#raveLogInfo('Tee is: '+repr(Tee)) 
                #time.sleep(2)
 
datafile.close()
       
 
