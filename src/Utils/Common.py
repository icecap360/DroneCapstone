from threading import Lock
from collections import deque 
from enum import Enum
 
App_PORT = 3002
App_buffSize=4096

def LogError(msg):
    print(msg +'!')
def LogDebug(msg):
    #print(msg +'.')
    return 
def LogMessage(msg):
    print(msg)

class SharedVal:
    def __init__(self, data):
        self.val = data
        self._lock = Lock()
    def get(self):
        self._lock.acquire()
        data = self.val
        self._lock.release()
        return data
    def set(self, data):
        self._lock.acquire()
        self.val = data
        self._lock.release()

class SharedQueue(SharedVal):
    def __init__(self):
        self.val = deque()
        self._lock = Lock()
    def popLeft(self):
        self._lock.acquire()
        if self.val:
            t = self.val.popleft()
        else:
            t = None
        self._lock.release()
        return t
    def append(self, data):
        self._lock.acquire()
        self.val.append(data)
        self._lock.release()

class UserErrorCode(Enum):
    NONE = 1
    DESIRED_LOCATION_INVALID = 2 
    NO_LOT_DETECTED = 3

class HealthStatusCode(Enum):
    HEALTHY = 1
    COMMUNICATION_LOST = 2 
    UNHEALTHY = 3
