#!/usr/bin/env python3
import sys
sys.path.append('/home/operator/DroneCapstone/src')
import rospy
from std_msgs.msg import String
from AlgorithmApp import AlgorithmApp
from VisionApp import VisionApp
from MapperApp import MapperApp
from PathPlanApp import PathPlanApp
from VisionApp import VisionApp
from Utils import DroneCamera
from TopicInterface import TopicInterface

if __name__ == "__main__":
    rospy.init_node('vision_app', anonymous=True)
    rate = rospy.Rate(10)

    algorithmApp = AlgorithmApp()
    visionApp = VisionApp()
    mapperApp = MapperApp()
    pathPlanApp = PathPlanApp()
    droneCamera = DroneCamera()
    topicInterface = TopicInterface()
    algorithmApp.init(visionApp, mapperApp,pathPlanApp, droneCamera, topicInterface )

    #fly in circle testcase
    #stateMachine.TestCircularMotion()

    #test launch 
    try:
        while True:
            algorithmApp.process()
            rate.sleep()
            #algorithmApp.rate.sleep(1)
    except KeyboardInterrupt:
        print('Keyboard expection')