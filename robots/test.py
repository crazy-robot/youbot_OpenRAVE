from openravepy import*
import time
env = Environment()
env.Load('anchor.robot.xml')
env.SetViewer('qtcoin')
x = -1
dx = 0.1

with env:
	robot = env.GetRobots()[0]
	robot.SetDOFValues([1,1,1,1,1])
	dx = dx + 0.1

while True:
	a = 1


		
         

