#!/usr/bin/env python3
import sys
sys.path.append('/home/operator/DroneCapstone/src')
import rospy
from std_msgs.msg import String
from AlgorithmApp import AlgorithmApp

if __name__ == "__main__":
    algorithmApp = AlgorithmApp()
    algorithmApp.init()

    #fly in circle testcase
    #stateMachine.TestCircularMotion()

    #test launch 
    try:
        while True:
            algorithmApp.process()
            #algorithmApp.rate.sleep(1)
    except KeyboardInterrupt:
        print('Keyboard expection')