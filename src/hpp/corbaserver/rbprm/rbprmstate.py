#!/usr/bin/env python
# Copyright (c) 2014 CNRS
# Author: Steve Tonneau
#
# This file is part of hpp-rbprm-corba.
# hpp-rbprm-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-manipulation-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-manipulation-corba.  If not, see
# <http://www.gnu.org/licenses/>.

from hpp.corbaserver.rbprm import Client as RbprmClient
from hpp.corbaserver import Client as BasicClient
from numpy import array


## Creates a state given an Id pointing to an existing c++ state
#
#  A RbprmDevice robot is a set of two robots. One for the 
#  trunk of the robot, one for the range of motion
class State (object):
    ## Constructor
    def __init__ (self, fullBody, sId, isIntermediate = False):
        self.cl = fullBody.client.rbprm.rbprm
        self.sId = sId
        self.isIntermediate = isIntermediate    
        self.fullBody = fullBody
    
    ## assert for case where functions can't be used with intermediate state
    def _cni(self):
        assert not self.isIntermediate, "method can't be called with intermediate state"
            
    ## Get the state configuration
    def q(self):
        self._cni()
        return self.cl.getConfigAtState(self.sId)
        
    ## Set the state configuration
    # \param q configuration of the robot
    # \return whether or not the configuration was successfully set
    def setQ(self, q):
        self._cni()
        return self.cl.setConfigAtState(self.sId, q)    > 0
        
    # \param state1 current state considered
    # \param limb name of the limb for which the request aplies
    # \return whether the limb is in contact at this state
    def isLimbInContact(self, limbname):
        if(self.isIntermediate):
            return self.cl.isLimbInContactIntermediary(limbname, self.sId) >0
        else:            
            return self.cl.isLimbInContact(limbname, self.sId) >0
                
    # 
    # \param state1 current state considered
    # \param limb name of the limb for which the request aplies
    # \return all limbs in contact at this state
    def getLimbsInContact(self):
        return [limbName for limbName in self.fullBody.limbNames if self.isLimbInContact(limbName)]
        
    ## Get the end effector position for a given configuration, assuming z normal
    # \param limbName name of the limb from which to retrieve a sample
    # \param configuration configuration of the robot
    # \return world position of the limb end effector given the current robot configuration.
    # array of size 4, where each entry is the position of a corner of the contact surface
    def getEffectorPosition(self, limbName):
        self._cni()
        return self.cl.getEffectorPosition(limbName,self.q())
     
    ## Get the end effector position for a given configuration, assuming z normal
    # \param limbName name of the limb from which to retrieve a sample
    # \param configuration configuration of the robot
    # \return world position of the limb end effector given the current robot configuration.
    # array of size 4, where each entry is the position of a corner of the contact surface
    def getContactPosAndNormals(self):
        if(self.isIntermediate):  
            rawdata = self.cl.computeContactPointsAtState(self.sId, 1)
        else:            
            rawdata = self.cl.computeContactPointsAtState(self.sId, 0) 
        return [[b[i:i+3] for i in range(0, len(b), 6)] for b in rawdata], [[b[i+3:i+6] for i in range(0, len(b), 6)] for b in rawdata]
     
    ## Get the end effector position for a given configuration, assuming z normal
    # \param limbName name of the limb from which to retrieve a sample
    # \param configuration configuration of the robot
    # \return world position of the limb end effector given the current robot configuration.
    # array of size 4, where each entry is the position of a corner of the contact surface
    def getContactPosAndNormalsForLimb(self, limbName):
        assert self.isLimbInContact(limbname), "in getContactPosAndNormals: limb " + limbName +  "is not in contact at  state" + str(stateId) 
        if(self.isIntermediate):  
            rawdata = cl.computeContactPointsAtStateForLimb(self.sId,1)
        else:            
            rawdata = cl.computeContactPointsAtStateForLimb(self.sId,0) 
        return [[b[i:i+3] for i in range(0, len(b), 6)] for b in rawdata], [[b[i+3:i+6] for i in range(0, len(b), 6)] for b in rawdata]
        
    
    ## Get position of center of mass
    def getCenterOfMass (self):
        q0 = fullBody.client.basic.robot.getCurrentConfig()
        fullBody.client.basic.robot.setCurrentConfig(self.q())
        c = fullBody.client.basic.robot.getComPosition()
        fullBody.client.basic.robot.setCurrentConfig(q0)
        return c
    
    
    ## Get the end effector position for a given configuration, assuming z normal
    # \param limbName name of the limb from which to retrieve a sample
    # \param configuration configuration of the robot
    # \return world position of the limb end effector given the current robot configuration.
    # array of size 4, where each entry is the position of a corner of the contact surface
    def getContactCone(self, friction):
        if(self.isIntermediate):  
            rawdata = self.cl.getContactIntermediateCone(self.sId,friction)
        else:            
            rawdata = self.cl.getContactCone(self.sId,friction) 
        H_h =  array(rawdata)
        return H_h[:,:-1], H_h[:, -1]
