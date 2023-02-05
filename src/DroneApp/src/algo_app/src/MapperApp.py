import rospy
from std_msgs.msg import Bool
from algo_app.msg import OccupancyMap 
from VisionApp import VisionApp

class MapperApp:
    def __init__(self, vApp:VisionApp):
        self.occupancyMap = OccupancyMap()
        self.visionApp = vApp
        #self.camera = Camera()
    def init(self):
        #self.camera.init()
        pass
    def process(self):
        # update occupancy map
        pass
    def getOccupancyMap(self):
        return self.occupancyMap
