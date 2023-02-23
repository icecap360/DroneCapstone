#! /usr/bin/env python3

import sys
sys.path.append('/home/operator/DroneCapstone/src')
from OperationManager import OperationManager
from Utils.MessageSocket import MessageSocket
from Utils import LogDebug
import time
from threading import Event, Thread

def infOpManLoop(stopNodeEvent, operationManager):
    while not stopNodeEvent.is_set():
        operationManager.process()
        LogDebug('Current State:'+operationManager.FSMState.name)
        operationManager.sleep(0.2)
    operationManager.close()

if __name__ == "__main__":
    operationManager = OperationManager(MessageSocket("DRONE"))
    operationManager.init()
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
    
    # try:
    #     t1 = Thread(target=infOpManLoop, args=(stopNodeEvent, operationManager))
    #     t1.start()
    #     operationManager.spin()
    # except KeyboardInterrupt:
    #     print('Keyboard expection')
    #     stopNodeEvent.set()
    # except:
    #     stopNodeEvent.set()



    #test configure 
    # operationManager.process()
    # operationManager.opAppInterface.addCommand({
    #     'Type':'Launch',
    #     'Mode':'Configure', 
    #     'MinHoverHeight':8,
    #     'DesiredHoverHeight':10,
    #     'MaxHoverHeight':12,
    #     })
    # operationManager.process()
    # operationManager.process()

