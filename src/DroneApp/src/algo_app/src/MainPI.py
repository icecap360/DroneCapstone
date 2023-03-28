#!/usr/bin/env python3
import sys, time
sys.path.append('/home/pi/DroneCapstone/src')
import rospy
from std_msgs.msg import String
from AlgorithmApp import AlgorithmApp
from VisionApp import VisionApp
from MapperApp import MapperApp
from PathPlanApp import PathPlanApp
from Utils import DroneCamera
from TopicInterface import TopicInterface
from threading import Event

if __name__ == "__main__":
    rospy.init_node('algo_app', anonymous=True)
    rate = rospy.Rate(10)
    stopNodeEvent = Event()
    algorithmApp = AlgorithmApp()
    visionApp = VisionApp()
    mapperApp = MapperApp()
    pathPlanApp = PathPlanApp()
    droneCamera = DroneCamera()
    topicInterface = TopicInterface()
    algorithmApp.init(visionApp, mapperApp,pathPlanApp, droneCamera, topicInterface )

    while not rospy.is_shutdown():
        algorithmApp.process()
        rate.sleep()
        time.sleep(1)