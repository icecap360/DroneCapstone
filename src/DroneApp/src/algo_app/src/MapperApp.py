# Author: Fady
# Date:December 2022
# Purpose: Occupancy Map creation module

import rospy
from std_msgs.msg import Bool
from algo_app.msg import OccupancyMap 

class MapperApp:
    def __init__(self):
        self.occupancyMap = OccupancyMap()
        self.health = True
    def init(self, visionApp, topicInterface):
        self.topicInterface = topicInterface
        self.visionApp = visionApp
    def process(self):
        self.occupancyMap.frame = 0
        if not self.visionApp.getHealth():
            # if vision app is not healthy, mapper app is also no healthy 
            self.health = False
            self.occupancyMap.Latitude = 0.0
            self.occupancyMap.Longitude = 0.0
        else:
            self.health = True
            self.occupancyMap.Latitude = self.visionApp.getImgLat()
            self.occupancyMap.Longitude = self.visionApp.getImgLong()
            self.occupancyMap.isParkLot = self.visionApp.getParkLotDet()
            self.occupancyMap.isNature = self.visionApp.getNatureDet()
            self.occupancyMap.isOccupied = self.visionApp.getOccupiedDet()
    def publish(self):
        self.topicInterface.occupancyMapPub.publish(self.getOccupancyMap())
        self.topicInterface.mapperAppHealth.publish(self.health)
    def getOccupancyMap(self):
        return self.occupancyMap
    def getHealth(self):
        return self.health
    