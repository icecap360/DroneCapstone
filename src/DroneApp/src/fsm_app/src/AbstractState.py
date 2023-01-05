import rospy
from abc import ABC, abstractmethod

class State(ABC):
    def __init__(self,context, command, name):
        self.context = context
        self.command=command
        self.raninit = False
        self.name = name
    
    ### The following are methods that the derived class implements
    @abstractmethod
    def init(self): #-> State:
        # executed once upon state entry
        self.raninit = True
    
    def during(self) :#-> State:
        # executed every frame, most states do nothing
        pass
    
    def exit(self) :#-> State:
        # executed every frame, most states do nothing
        pass

    @abstractmethod
    def evalNewState(self):
        # return None if next state is same as before
        return None

    ### These are methods that are called by the state machine
    def process(self) :#-> State:
        if not self.raninit:
            self.init()
        else:
            self.during()

    def transitionNextState(self):
        newState = self.evalNewState()
        if newState == None:
            return self
        else:
            self.exit()
            return newState

    def __str__(self):
        return self.name

    # ### These methods are optional
    # def processUserCommand(self, command):
    #     # optional to implement and use
    #     raise NotImplemented