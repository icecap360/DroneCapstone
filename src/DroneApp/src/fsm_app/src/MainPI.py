#! /usr/bin/env python3

import sys
sys.path.append('/home/pi/DroneCapstone/src')
from OperationManager import OperationManager
from Utils.MessageSocket import MessageSocket
from Utils import LogDebug
import time
from threading import Event, Thread
from TopicInterface import TopicInterface
from ServiceInterface import ServiceInterface

def infOpManLoop(stopNodeEvent, operationManager):
    while not stopNodeEvent.is_set():
        operationManager.process()
        LogDebug('Current State:'+operationManager.FSMState.name)
        operationManager.sleep(0.2)
    operationManager.close()

if __name__ == "__main__":
    operationManager = OperationManager()
    msgSock = MessageSocket("DRONE", "")
    topicInterf = TopicInterface()
    serviceInterf = ServiceInterface()
    operationManager.init(msgSock, topicInterf, serviceInterf)
    stopNodeEvent = Event()

    #fly in circle testcase
    #operationManager.TestCircularMotion()

    try:
        while not stopNodeEvent.is_set():
            operationManager.process()
            LogDebug('Current State:'+operationManager.FSMState.name)
            operationManager.sleep(0.5)
    except KeyboardInterrupt:
        LogDebug('Keyboard expection')
        stopNodeEvent.set()
        operationManager.close()


