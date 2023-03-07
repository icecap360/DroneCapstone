import rospy
from std_msgs.msg import Bool
from Segmenters import ParkingLotSegmenter, NatureSegmenter
from DataSetManager import DataSetManager
from Classifier import Classifier

class VisionApp:
    def __init__(self):
        self.parkLotDetected = False
        self.health = True

        self.parkLotSegmenter = ParkingLotSegmenter()
        self.natureSegmenter = NatureSegmenter()
        self.classifier = Classifier()
        self.parkLotDetected = None
        self.natureDetected = None
        self.occupiedDetected = None
        #self.camera = Camera()
    def init(self, droneCamera, topicInterface):
        self.droneCamera = droneCamera
        self.topicInterface = topicInterface
        #self.camera.init()
    def process(self):
        # image processing code goes here  
        self.droneCamera.getImage()
        self.processImage(self.droneCamera.image)
        
        # self.parkLotDetected = True
    
    def publish(self):
        self.topicInterface.parkLotDetectedPub.publish(self.getParkLotDet()) #always publish this, as a default transition depends on this
        self.topicInterface.visionAppHealth.publish(self.health)
    def getParkLotDet(self):
        return self.parkLotDetected
    def getHealth(self):
        return self.health
    def processImage(self, img, cropImage=True, exeParkLot=True, exeNature=True):
        if exeParkLot:
            self.parkLotSegmenter.process(img, cropImage)
            self.parkLotDetected  = self.classifier.classify(self.parkLotSegmenter.getResult())
        if exeNature:
            self.natureSegmenter.process(img, cropImage)
            self.natureDetected  = self.classifier.classify(self.natureSegmenter.getResult())
        if exeNature and exeParkLot:
            self.occupiedDetected = (not self.natureDetected) and (not self.parkLotDetected)

