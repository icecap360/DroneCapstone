import rospy
from abc import ABC, abstractmethod

class State(ABC):
    def __init__(self,context, command, name):
        self.context = context
        self.command=command
        self.raninit = False
        self.name = name
    
    @abstractmethod
    def init(self): #-> State:
        # executed once upon state entry
        # returns next state
        self.raninit = True
        return self

    def processUserCommand(self, command):
        raise NotImplemented
    
    @abstractmethod
    def during(self) :#-> State:
        # executed once upon state entry
        # returns next state
        pass

    def process(self) :#-> State:
        if not self.raninit:
            return self.init()
        return self.during()
    
    def __str__(self):
        return self.name

