#! /usr/bin/env python3

# Author: Ali
# Date:December 2022
# Purpose: Main FSM node for the drone platform.

import sys
sys.path.append('/home/pi/DroneCapstone/src')
from OperationManager import OperationManager
from Utils.MessageSocket import MessageSocket
from Utils import LogDebug
import time
from threading import Event, Thread
from TopicInterface import TopicInterface
from ServiceInterface import ServiceInterface

if __name__ == "__main__":
    operationManager = OperationManager()
    msgSock = MessageSocket("DRONE", "")
    topicInterf = TopicInterface()
    serviceInterf = ServiceInterface()
    operationManager.init(msgSock, topicInterf, serviceInterf)

    # create node stop event that will be used to safely close operation manager
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


