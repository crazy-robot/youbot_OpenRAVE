
from openravepy import *
from numpy import *
import time
env = Environment() # create the environment
env.Load('../environment/youbotsimple.env.xml')
#env.SetViewer('qtcoin')
robot = env.GetRobots()[0] # get the first robot
manip = robot.SetActiveManipulator('arm') # set the manipulator to leftarm
base = array([[0,0,0,0],[0,0,0,0],[0,0,-1,0],[0,0,0,1]])

with env: # lock environment
    lower,upper = robot.GetDOFLimits(manip.GetArmIndices())
    print lower+random.rand(len(lower))*(upper-lower)
    robot.SetDOFValues([0,0,0,0,0,0])
    with robot: # save robot state
	#robot.SetDOFValues([0,0,0,0,0],manip.GetArmIndices()) # set the current solution
	Tbaseinv = linalg.inv(manip.GetBase().GetTransform())
   	Tee =dot(Tbaseinv, manip.GetEndEffectorTransform())
	#Tee = manip.GetEndEffectorTransform()
	#Tee = dot(Tee,base)
    	env.UpdatePublishedBodies() # allow viewer to update new robot
	time.sleep(10)

raveLogInfo('Tee is: '+repr(Tee))

while True:
	a = 1
 
