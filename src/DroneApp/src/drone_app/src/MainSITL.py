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
    while True:
        stateMachine.process()
        print(stateMachine.State, stateMachine.arduInfoReader.getHealthStatus())
        stateMachine.sleep(2)


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


    
