import rospy
from abc import ABC, abstractmethod

class State(ABC):
    def __init__(self,context, command, name):
        self.context = context
        self.command=command
        self.ranInit = False
        self.name = name
    
    @abstractmethod
    def Init(self): #-> State:
        # executed once upon state entry
        # returns next state
        self.ranInit = True
        return self

    def _ProcessUserCommand(self):
        # optional to implement and call
        rospy.loginfo('Abstract State processUserCommand')
        pass
    
    @abstractmethod
    def During(self) :#-> State:
        # executed once upon state entry
        # returns next state
        pass

    def Process(self) :#-> State:
        if not self.ranInit:
            return self.Init()
        return self.During()
    
    def __str__(self):
        return self.name


class FlightState(State):
    def process(self):
        super().__init__()
    
    @abstractmethod
    def Init(self): #-> State:
        # executed once upon state entry
        # returns next state
        self.ranInit = True
        return self

    def _ProcessUserCommand(self):
        # optional to implement and call
        rospy.loginfo('Abstract State processUserCommand')
        pass
    
    @abstractmethod
    def During(self) :#-> State:
        # executed once upon state entry
        # returns next state
        pass

    def Process(self) :#-> State:
        if not self.ranInit:
            return self.Init()
        return self.During()
    
    def __str__(self):
        return self.name