#! /usr/bin/env python3

import sys
sys.path.append('/home/operator/DroneCapstone/src')
from OperationManager import OperationManager
import Utils.Sockets as Sockets

if __name__ == "__main__":
    operationManager = OperationManager(Sockets.DroneSocket())
    operationManager.init()

    #fly in circle testcase
    #operationManager.TestCircularMotion()

    #test launch 
    try:
        while True:
            operationManager.process()
            print('Current State:',operationManager.FSMState)
            operationManager.sleep(0.2)
    except KeyboardInterrupt:
        print('Keyboard expection')


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

