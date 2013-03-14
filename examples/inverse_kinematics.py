"""Shows how to get all 6D IK solutions.
"""
from openravepy import *
from numpy import *
import time
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
env.Load('../environment/youbotsimple.env.xml')
robot = env.GetRobots()[0] # get the first robot

manip = robot.SetActiveManipulator('arm') # set the manipulator to leftarm
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.TranslationDirection5D)
if not ikmodel.load():
    ikmodel.autogenerate()

#-0.8414351501256625,0.3524602332302655,0.4095835349770189 -2.957664177109279,-0.9207600901162143,0.3069180475276921
with env: # lock environment
    lower,upper = robot.GetDOFLimits(manip.GetArmIndices())
    print lower+random.rand(len(lower))*(upper-lower)
    robot.SetDOFValues(lower+random.rand(len(lower))*(upper-lower),manip.GetArmIndices())
    ikparam = manip.GetIkParameterization(IkParameterization.Type.TranslationDirection5D)
    sols = manip.FindIKSolutions(ikparam,IkFilterOptions.CheckEnvCollisions)
    if sols is not None:
    	with robot: # save robot state
	    for i in range(0,sols.shape[0]):
            	print sols[i]
            	robot.SetDOFValues(sols[i],manip.GetArmIndices()) # set the current solution
    	   	Tee = manip.GetEndEffectorTransform()
    	    	env.UpdatePublishedBodies() # allow viewer to update new robot
                raveLogInfo('Tee is: '+repr(Tee))
    	    	time.sleep(10)
    if sols is None:
	print "IK could not find solutions"
   
 
