#! /usr/bin/env python3

import sys
sys.path.append('/home/operator/DroneCapstone/src')
from StateMachine import StateMachine
import Utils.Sockets as Sockets

if __name__ == "__main__":
    stateMachine = StateMachine(Sockets.DroneSocket())
    stateMachine.init()

    #fly in circle testcase
    #stateMachine.TestCircularMotion()

    #test launch 
    try:
        while True:
            stateMachine.process()
            print('Current State:',stateMachine.State)
            stateMachine.sleep(0.2)
    except KeyboardInterrupt:
        print('Keyboard expection')


    #test configure 
    # stateMachine.process()
    # stateMachine.opAppInterface.addCommand({
    #     'Type':'Launch',
    #     'Mode':'Configure', 
    #     'MinHoverHeight':8,
    #     'DesiredHoverHeight':10,
    #     'MaxHoverHeight':12,
    #     })
    # stateMachine.process()
    # stateMachine.process()

