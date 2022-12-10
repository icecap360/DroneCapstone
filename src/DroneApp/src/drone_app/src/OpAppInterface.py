from collections import deque 
from threading import Semaphore

class OpAppInterface:
    def __init__(self):
        self.semCommands = Semaphore()
        self.commands = deque([]) 
    
    def getNextCommand(self):
        self.semCommands.acquire()
        if self.commands:
            t = self.commands.popleft()
        else:
            t = None
        self.semCommands.release()
        return t

    def addCommand(self, command):
        self.semCommands.acquire()
        self.commands.append(command)
        self.semCommands.release()
    