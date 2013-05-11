"""Shows how to solve IK for two robot arms  simultaneously
"""
from openravepy import *
from numpy import *
from itertools import izip
import time
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
env.Load('../environment/dualarmmanipulation.env.xml')

# get the first robot
robot=env.GetRobots()[0]
manip = robot.SetActiveManipulator('arm') # set the manipulator to leftarm
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.TranslationDirection5D)
if not ikmodel.load():
    ikmodel.autogenerate()
manip1 = robot.SetActiveManipulator('arm1') # set the manipulator to leftarm
ikmodel1 = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.TranslationDirection5D)
if not ikmodel1.load():
    ikmodel1.autogenerate()
# create the dual-arm ik solver
dualsolver = misc.MultiManipIKSolver([robot.GetManipulator('arm'), robot.GetManipulator('arm1')])
#robot.SetDOFValues([-1.57,1,-1,0,1.57],manip.GetArmIndices())
#robot.SetDOFValues([-1.57,1,-1,0,1.57],manip1.GetArmIndices())
taskprob = interfaces.TaskManipulation(robot)
#taskprob.ReleaseFingers()
body=env.GetKinBody('Object1')
while True:
	for itry in range(5):
	    with env:
		Tbody=body.GetTransform()
		ab = body.ComputeAABB().extents()
		halfwidth= ab[1] #this is y
		#.04 is just half the thickness of the EEF
		TRightGrasp= dot(Tbody,array([[0, 0, -1, 0],[1, 0, 0, (halfwidth+.04)],[0, -1, 0, 0 ],[0, 0, 0, 1]]))
		# to determine the grasp for the eef given the transform of the object
		TLeftGrasp= dot(Tbody,array([[0, 0, -1, 0],[-1, 0, 0, -(halfwidth+.04)],[0, 1, 0, 0],[0, 0, 0, 1]]))
		TRightGrasp = manip.GetEndEffectorTransform()
		TLeftGrasp = manip1.GetEndEffectorTransform()
		solutions = dualsolver.YouBotMultiIKDemo(filteroptions=IkFilterOptions.IgnoreCustomFilters)
		#lower,upper = robot.GetDOFLimits(manip.GetArmIndices())
                #solutions= lower+ random.rand(len(lower))*(upper-lower)
		if solutions is not None:
		    for manip,solution in izip(dualsolver.manips, solutions):
		        robot.SetDOFValues(solution,manip.GetArmIndices())
		#robot.Grab(body)
		
		    
	    #time.sleep(0.2)


