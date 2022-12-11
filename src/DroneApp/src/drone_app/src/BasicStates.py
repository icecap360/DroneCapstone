import rospy
from abc import ABC
from AbstractStates import State
from TakeoffStates import Takeoff
import json

class Configure(State):
    def __init__(self,context, minHoverHeight, desiredHoverHeight, maxHoverHeight):
        self.minHoverHeight = minHoverHeight
        self.desiredHoverHeight = desiredHoverHeight
        self.maxHoverHeight = maxHoverHeight
        super().__init__(context, 'Configure')
    
    def Init(self) -> State:
        self.context.setAndSaveParams(self.minHoverHeight, self.desiredHoverHeight, self.maxHoverHeight)
        return super().Init()
    
    def During(self) -> State:
        rospy.loginfo('launch during')
        pass

class Idle(State):
    def __init__(self,context):
        super().__init__(context, 'Idle')
    
    def Init(self) -> State:
        return super().Init()
    
    def During(self) -> State:        
        command = self.context.opAppInterface.getMessage()

        if command != None:
            if command['Type'] == 'Launch' and command['Mode'] != 'Configure':
                return Takeoff(self.context)
            elif command['Type'] == 'Launch' and command['Mode'] == 'Configure':
                return Configure(self.context, command['MinHoverHeight'], 
                    command['DesiredHoverHeight'], command['MaxHoverHeight'])
            else:
                return self
        else:
            return self
