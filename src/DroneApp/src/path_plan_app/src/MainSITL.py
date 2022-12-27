#!/usr/bin/env python3
import sys
sys.path.append('/home/operator/DroneCapstone/src')
import rospy
from std_msgs.msg import String
from PathPlanApp import PathPlanApp

if __name__ == "__main__":
    pathPlanApp = PathPlanApp()
    pathPlanApp.init()

    #fly in circle testcase
    #stateMachine.TestCircularMotion()

    #test launch 
    try:
        while True:
            pathPlanApp.process()
            #pathPlanApp.rate.sleep(1)
    except KeyboardInterrupt:
        print('Keyboard expection')