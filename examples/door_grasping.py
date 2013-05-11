from openravepy import *
import numpy, time
env=Environment()
env.Load('../environment/door.env.xml')
env.SetViewer('qtcoin')
robot = env.GetRobots()[0]
body = env.GetBodies()[0]
frame = body.GetLink("frame")
door = body.GetLink("door")
door.Enable(False)
frame.Enable(False)
target = env.GetKinBody('door')
recorder = RaveCreateModule(env,'viewerrecorder')
env.AddModule(recorder,'')
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()
gmodel = databases.grasping.GraspingModel(robot,target)
if not gmodel.load():
    gmodel.autogenerate()

manip = robot.SetActiveManipulator('arm')
Tstart = manip.GetEndEffectorTransform()
door = env.GetKinBody('door')
x = raw_input('What do you want to do?\n')
print "You want me to \n" +x
time.sleep(3)
door.Enable(True)
frame.Enable(True)
if x == "1":
	print "Retrieving Semantics of the type of door handle"
	time.sleep(3)
	print "Recogonized as type1 handle..Identifying Constraints"
	time.sleep(3)
	print "Finding Valid Grasps"
	validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=1)
	gmodel.moveToPreshape(validgrasps[0])
	Tgoal = gmodel.getGlobalGraspTransform(validgrasps[0],collisionfree=True)
	basemanip = interfaces.BaseManipulation(robot)
	basemanip.MoveToHandPosition(matrices=[Tgoal])
	robot.WaitForController(0)
	#taskmanip = interfaces.TaskManipulation(robot)
	#taskmanip.CloseFingers()
        #basemanip.MoveToHandPosition(matrices=[Tstart])
	robot.WaitForController(0)
print "Handle Grasped Firmly.."
x = raw_input("What else ? \n")
print "You want me to \n" +x
time.sleep(3)
with env:
if x == "2":
	with robot:
		print "Retrieving Semantics of type of unlatching"
		time.sleep(3)
		print "Door is unlocked condition"
		with door:
			door.SetDOFValues([0,1])
	#basemanip.MoveToHandPosition(matrices=[Tstart])
	#robot.WaitForController(0)
	
	

while True:
	a = 1
