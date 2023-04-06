#!/usr/bin/env python3

# Author: Ali
# Date:December 2022
# Purpose: Main algorithm node for testing on SITL

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
    # initialize ROS node
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

    while not rospy.is_shutdown(): # while ROS has not shutdown this node 
        algorithmApp.process()
        rate.sleep()
