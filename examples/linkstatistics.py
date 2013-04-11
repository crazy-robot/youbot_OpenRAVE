from openravepy import *
import time
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
env.Load('../environment/youbot_move.env.xml') # load a scene
robot = env.GetRobots()[0] # get the first robot
robot.SetActiveManipulator('arm')
goal = [0,1.5,1.5,-2,0]

# normal planning
manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
starttime = time.time()
manipprob.MoveManipulator(goal=goal,execute=False,outputtrajobj=True)
raveLogInfo('non-linkstatistics planning time: %fs'%(time.time()-starttime))

# using link statistics
lmodel=databases.linkstatistics.LinkStatisticsModel(robot)
if not lmodel.load():
    lmodel.autogenerate()
    
lmodel.setRobotResolutions(0.01) # set resolution given smallest object is 0.01m
lmodel.setRobotWeights() # set the weights for planning
starttime = time.time()
traj=manipprob.MoveManipulator(goal=goal,execute=False,outputtrajobj=True)
raveLogInfo('linkstatistics planning time: %fs'%(time.time()-starttime))
robot.GetController().SetPath(traj)
robot.WaitForController(0)
while True:
	a =1

