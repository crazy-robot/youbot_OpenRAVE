#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) Praveen Ramanujam (pramanuj86@gmail.com)
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

from openravepy import *
from numpy import *
import time

class youBot:
	
	 def __init__(self,robot):
		self.envreal = robot.GetEnv()
		self.robot = robot
		self.manip = self.robot.GetActiveManipulator()
		self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.TranlationDirection5D)
		if not self.ikmodel.load():
		    self.ikmodel.autogenerate()
		self.basemanip = interfaces.BaseManipulation(self.robot,plannername=None)
		print "youBot Initialized"


	 def simpleManipulation(self):
	 ''' Need to have inverse kinematics'''
		print "Parameterization of target required"



def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    env.UpdatePublishedBodies()
    time.sleep(0.1) # give time for environment to update
    self = youBot(robot)
    self.simpleManipulation()

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='RRT motion planning with constraints on the robot end effector.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='/home/student/rnd2/devel/ros_stacks/sandbox/urdf_collada/bin/lab1.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
	
