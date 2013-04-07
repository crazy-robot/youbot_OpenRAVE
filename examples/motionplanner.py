from openravepy import *
import time
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
env.Load('data/pr2test1.env.xml') # load a scene
robot = env.GetRobots()[0] # get the first robot
robot.SetActiveManipulator('leftarm_torso')
goal = [ 0.24865706, 0.09362862, 0, 2.21558089, -1.00901245, -1.18879056, -0.74486442, 0]

# normal planning
manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
starttime = time.time()
manipprob.MoveManipulator(goal=goal,execute=False)
raveLogInfo('non-linkstatistics planning time: %fs'%(time.time()-starttime))
