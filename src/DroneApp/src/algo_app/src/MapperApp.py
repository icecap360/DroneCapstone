import rospy
from std_msgs.msg import Bool
from algo_app.msg import OccupancyMap 
from VisionApp import VisionApp

class MapperApp:
    def __init__(self):
        self.occupancyMap = OccupancyMap()
        self.health = True
        #self.camera = Camera()
    def init(self, visionApp, topicInterface):
        self.topicInterface = topicInterface
        self.visionApp = visionApp
        #self.camera.init()
    def process(self):
        # update occupancy map
        pass
    def publish(self):
        self.topicInterface.occupancyMapPub.publish(self.getOccupancyMap())
        self.topicInterface.mapperAppHealth.publish(self.health)
    def getOccupancyMap(self):
        return self.occupancyMap
    def getHealth(self):
        return self.health