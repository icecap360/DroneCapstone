import rospy
from std_msgs.msg import Bool

class VisionApp:
    def __init__(self):
        self.parkLotDetected = False
        self.health = True
        #self.camera = Camera()
    def init(self, droneCamera, topicInterface):
        self.droneCamera = droneCamera
        self.topicInterface = topicInterface
        #self.camera.init()
    def process(self):
        # image processing code goes here  
        
        #update parkLotDetected
        self.parkLotDetected = True
    
    def publish(self):
        self.topicInterface.parkLotDetectedPub.publish(self.getParkLotDet()) #always publish this, as a default transition depends on this
        self.topicInterface.visionAppHealth.publish(self.health)
    def getParkLotDet(self):
        return self.parkLotDetected
    def getHealth(self):
        return self.health
