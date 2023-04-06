#!/usr/bin/env python3

# Author: Ali
# Date:December 2022
# Purpose: Main algorithm node for drone

import sys, time
sys.path.append('/home/pi/DroneCapstone/src')
import rospy
from std_msgs.msg import String
from AlgorithmApp import AlgorithmApp
from MapperApp import MapperApp
from PathPlanApp import PathPlanApp
from Utils import DroneCamera, VisionAppPI
from TopicInterface import TopicInterface
from threading import Event

if __name__ == "__main__":
    # initialize ROS node
    rospy.init_node('algo_app', anonymous=True)
    rate = rospy.Rate(10)

    stopNodeEvent = Event()
    algorithmApp = AlgorithmApp()
    visionApp = VisionAppPI()
    mapperApp = MapperApp()
    pathPlanApp = PathPlanApp()
    droneCamera = DroneCamera()
    topicInterface = TopicInterface()
    algorithmApp.init(visionApp, mapperApp,pathPlanApp, droneCamera, topicInterface )

    while not rospy.is_shutdown(): # while ROS has not shitdown this node 
        algorithmApp.process()
        rate.sleep()
        time.sleep(0.5)
    
    # close camera upon end
    droneCamera.close()