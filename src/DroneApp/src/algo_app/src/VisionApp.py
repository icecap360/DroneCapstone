import rospy
from std_msgs.msg import Bool

class VisionApp:
    def __init__(self):
        self.parkLotDetected = False
        #self.camera = Camera()
    def init(self):
        #self.camera.init()
        pass
    def readCamera(self):
        # this function is emant to be called before process
        #self.camera.read()
        pass
    def process(self):
        # image processing code goes here  
        self.parkLotDetected = True
        pass

