#! /usr/bin/env python3

from OperationManager import OperationManager
from OpAppInterface import OpAppInterface

if __name__ == "__main__":
    operationManager = OperationManager(OpAppInterface())
    operationManager.init()

    #fly in circle testcase
    operationManager.TestCircularMotion()

    #test launch (sitl only right now)
    # operationManager.process()
    # operationManager.opAppInterface.addCommand({'Type':'Launch','Mode':'Normal'})
    # operationManager.process()
    # operationManager.process()

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


    
