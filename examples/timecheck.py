from openravepy import *
from numpy import *
import time
from openravepy.misc import InitOpenRAVELogging
import matplotlib.pyplot as plt
from scipy import*

InitOpenRAVELogging() 
env = Environment()
env.Load('/home/student/Downloads/robots/youbot1.robot.xml')
robot = env.GetRobots()[0] 

noik = 0
t=[]
manip = robot.SetActiveManipulator('arm') # set the manipulator to leftarm
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.TranslationDirection5D)
if not ikmodel.load():
    ikmodel.autogenerate()

for check in range (0,100):
    with env: # lock environment
        starttime = time.time()
        lower,upper = robot.GetDOFLimits(manip.GetArmIndices())
        a = lower+random.rand(len(lower))*(upper-lower)
        robot.SetDOFValues(a,manip.GetArmIndices())
	FK = manip.GetEndEffectorTransform()
        ikparam = manip.GetIkParameterization(IkParameterization.Type.TranslationDirection5D)
        sols = manip.FindIKSolutions(ikparam,IkFilterOptions.IgnoreCustomFilters)
	t.append(time.time()-starttime)
	
	raveLogInfo('Inverse Kinematics time: %fs'%(t[check]))
	if sols.shape[0] < 1:
		noik = noik+1
	else :
	 for i in range(0,1):
            with robot:
                robot.SetDOFValues(sols[i],manip.GetArmIndices()) # set the current solution
    	    	IK = manip.GetEndEffectorTransform()
		e = numpy.linalg.norm(FK-IK)
		if e > 0.01:
                	raveLogInfo('Computed Error: %f'%(e))
			print robot.GetDOFValues()

print noik
y = linspace(1,100,100)
plt.plot(y,t)
plt.xlabel('Number of tests peformed')
plt.ylabel('Time taken for correspoding test')
plt.show()

       
 
