#! /usr/bin/env python3

from StateMachine import StateMachine
from OpAppInterface import OpAppInterface

if __name__ == "__main__":
    stateMachine = StateMachine(OpAppInterface())
    stateMachine.SetupRosNode()

    #fly in circle testcase
    stateMachine.TestCircularMotion()

    #test launch (sitl only right now)
    # stateMachine.process()
    # stateMachine.opAppInterface.addCommand({'Type':'Launch','Mode':'Normal'})
    # stateMachine.process()
    # stateMachine.process()

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


    
