#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Shows how to use RRTs for navigation planning by setting affine degrees of freedom.

.. examplepre-block:: simplenavigation
  :image-width: 400

.. examplepost-block:: simplenavigation
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time,numpy
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class SimpleNavigationPlanning:
    def __init__(self,robot,randomize=False,dests=None,switchpatterns=None):
        self.env = robot.GetEnv()
        self.robot = robot
        self.cdmodel = databases.convexdecomposition.ConvexDecompositionModel(self.robot)
        if not self.cdmodel.load():
            self.cdmodel.autogenerate()
        self.basemanip = interfaces.BaseManipulation(self.robot)
    def performNavigationPlanning(self):
        # find the boundaries of the environment
        with self.env:
            envmin = []
            envmax = []
            for b in self.env.GetBodies():
                ab = b.ComputeAABB()
                envmin.append(ab.pos()-ab.extents())
                envmax.append(ab.pos()+ab.extents())
            abrobot = self.robot.ComputeAABB()
            envmin = numpy.min(array(envmin),0)+abrobot.extents()
            envmax = numpy.max(array(envmax),0)-abrobot.extents()
        bounds = array(((envmin[0],envmin[1],-pi),(envmax[0],envmax[1],pi)))
	t = self.env.GetBodies()[1]
	g = array([[ 0.53320741,  0.71557224, -0.45127179,  3.20530494], [-0.80691723,  0.59041347, -0.01721968, -0.9592293 ],[ 0.25411502,  0.37332064,0.89222041,  0.48222358],[ 0, 0, 0,  1]])	
	t.SetTransform(g)
        while True:
            with self.env:
                self.robot.SetAffineTranslationLimits(envmin,envmax)
                self.robot.SetAffineTranslationMaxVels([0.5,0.5,0.5])
                self.robot.SetAffineRotationAxisMaxVels(ones(4))
                self.robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
                # pick a random position
                with self.robot:
                    while True:
                        goal = bounds[0,:]+random.rand(3)*(bounds[1,:]-bounds[0,:])
                        self.robot.SetActiveDOFValues(goal)
                        if not self.env.CheckCollision(self.robot):
                            break
            goal = [2.8,-1.3,1.5765023]
            print 'planning to: ',goal
            # draw the marker
            
            center = r_[goal[0:2],0.2]
            xaxis = 0.5*array((cos(goal[2]),sin(goal[2]),0))
            yaxis = 0.25*array((-sin(goal[2]),cos(goal[2]),0))
            h = self.env.drawlinelist(transpose(c_[center-xaxis,center+xaxis,center-yaxis,center+yaxis,center+xaxis,center+0.5*xaxis+0.5*yaxis,center+xaxis,center+0.5*xaxis-0.5*yaxis]),linewidth=5.0,colors=array((0,1,0)))
           
            if self.basemanip.MoveActiveJoints(goal=goal,maxiter=3000,steplength=0.1) is None:
                print 'retrying...'
                continue
            print 'waiting for controller'
            self.robot.WaitForController(0)
            taskprob = interfaces.TaskManipulation(self.robot)
            taskprob.ReleaseFingers()
            while True:
                manip = self.robot.SetActiveManipulator('arm') # set the manipulator to leftarm
                ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterization.Type.TranslationDirection5D)
                if not ikmodel.load():
                    ikmodel.autogenerate()
                with self.env: # lock environment
                    lower,upper = self.robot.GetDOFLimits(manip.GetArmIndices())
                    #a = lower+random.rand(len(lower))*(upper-lower)
                    a = array([-0.88664706, 0.80421693, 0.18288266, 0.20111085, -2.86492377])
                    self.robot.SetDOFValues(a,manip.GetArmIndices())
                    ikparam =  IkParameterization('1442840583 -0.5335666042260692 0.8069032587497339 -0.2534044393330962 3.125495570275616 -1.007323359687846 0.4969689507536642 ')
                    ikparam = manip.GetIkParameterization(IkParameterization.Type.TranslationDirection5D)
                    sols = manip.FindIKSolutions(ikparam,IkFilterOptions.IgnoreCustomFilters)
                    Tee = manip.GetEndEffectorTransform()
                    for i in range(0,sols.shape[0]):
                        with self.robot:
                            a = sols[i]
                            print a
                            self.robot.SetDOFValues(sols[i],manip.GetArmIndices()) 
                            self.env.UpdatePublishedBodies()
                    self.robot.Grab(t)
                    #taskprob.CloseFingers()
                    with self.robot:
                        goal = [2.8,-1.3,0]
                        print 'planning to: ',goal
                        # draw the marker
                
                        center = r_[goal[0:2],0.2]
                        xaxis = 0.5*array((cos(goal[2]),sin(goal[2]),0))
                        yaxis = 0.25*array((-sin(goal[2]),cos(goal[2]),0))
                        h = self.env.drawlinelist(transpose(c_[center-xaxis,center+xaxis,center-yaxis,center+yaxis,center+xaxis,center+0.5*xaxis+0.5*yaxis,center+xaxis,center+0.5*xaxis-0.5*yaxis]),linewidth=5.0,colors=array((0,1,0)))
               
                        if self.basemanip.MoveActiveJoints(goal=goal,maxiter=3000,steplength=0.1) is None:
                            print 'retrying...'
                            continue
                        print 'waiting for controller'
                        self.robot.WaitForController(0)
                   
                while True:
                            a = 1

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    env.UpdatePublishedBodies()
    time.sleep(0.1) # give time for environment to update
    self = SimpleNavigationPlanning(robot)
    self.performNavigationPlanning()

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Simple navigation planning using RRTs.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='../environment/youbot_base_manipulation.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
