from threading import Semaphore
from TopicInterface import TopicInterface
from VisionApp import VisionApp
from MapperApp import MapperApp
from PathPlanApp import PathPlanApp
from Utils.Common import LogMessage
from geometry_msgs.msg import PoseStamped

class AlgorithmApp:
    def __init__(self):
        pass

    def init(self, vapp, mapp, ppapp, dronecam, topinterf):
        self.visionApp = vapp
        self.mapperApp = mapp
        self.pathPlanApp = ppapp
        self.droneCamera = dronecam
        self.topicInterface = topinterf
        self.droneCamera.init()
        # create segmented image publisher
        # create occupancy map publisher
        self.visionApp.init( self.droneCamera, self.topicInterface)
        self.mapperApp.init(self.visionApp, self.topicInterface)
        self.pathPlanApp.init(self.pathPlanApp, self.topicInterface)
    
    def pathplan(self):
        pass
    def process(self):
        # Run the algorithms
        self.visionApp.process()
        self.mapperApp.process()
        self.pathPlanApp.process()
        
        self.visionApp.publish()
        self.mapperApp.publish()
        self.pathPlanApp.publish()

