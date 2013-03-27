#!/usr/bin/env python 

from __future__ import with_statement # for python 2.5
__author__= 'Praveen Ramanujam'

import time

from openravepy import *
from numpy import *

def waitrobot(robot):
    while not robot.GetController().IsDone():
	time.sleep(0.01)

def main():
#setup the basic environment
   
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('../environment/youbot_base_manipulation.env.xml')
    robot = env.GetRobots()[0]

    count = 1
    env.UpdatePublishedBodies()
    time.sleep(0.1)

#create convex decomposition
    cdmodel = databases.convexdecomposition.ConvexDecompositionModel(robot)
    if not cdmodel.load():
        cdmodel.autogenerate()
   

    basemanip = interfaces.BaseManipulation(robot)
    goal = [-3.03459345,-0.58014151,0.34427884]
    #target = env.GetKinBody('TibitsBox1')
    #print target    
    #goal = [-2.49138767,1.64301229,1.57298522] #in x,y and angle for 2D plane
    
    print 'move robot base to target'
    
    while True:
        count = count +1
        with env:
             robot.SetAffineTranslationMaxVels([0.5,0.5,0.5])
             robot.SetAffineRotationAxisMaxVels(ones(4))
             robot.SetActiveDOFs([],Robot.DOFAffine.X|Robot.DOFAffine.Y|Robot.DOFAffine.RotationAxis,[0,0,1])
             robot.GetActiveDOF()
             
             if env.CheckCollision(robot):
                print "Goal in collision"
             basemanip.MoveActiveJoints(goal=goal,maxiter=3000,steplength=0.1)
        robot.WaitForController(0)
        if count >2 :
            break

        goal = [0,0,0]
    
             #basemanip.MoveActiveJoints(goal=[0,0,0],maxiter=3000,steplength=0.1)
             #robot.WaitForController(0)
             #basemanip.MoveActiveJoints(goal=[0,0,0],maxiter=3000,steplength=0.15,maxtries=1)
             #waitrobot(robot)

#from optparse import OptionParser
#from openravepy import OpenRAVEGlobalArguments, with_destroy
#@with_destroy
def run(args=None):
    main()
    
    
if __name__ == "__main__":
    run() 
