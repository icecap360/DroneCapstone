import numpy as np
import cv2
from matplotlib import pyplot as plt
from PIL import Image
from abc import ABC
from Segmenters import ParkingLotSegmenterPI, NatureSegmenter, ParkingLotSegmenterPC
from Classifier import Classifier

class VisionAppPI:
	def __init__(self) -> None:
		self.parkLotSegmenter = ParkingLotSegmenterPI()
		self.natureSegmenter = NatureSegmenter()
		self.classifier = Classifier()
		self.parkLotDetected = None
		self.natureDetected = None
		self.occupiedDetected = None
	def init(self, droneCamera, topicInterface):
		self.droneCamera = droneCamera
		self.topicInterface = topicInterface
	def process(self):
		self.droneCamera.read()
		self.processImage(self.droneCamera.image)
	def processImage(self, img, cropImage=True, exeParkLot=True, exeNature=True):
		if exeParkLot:
			self.parkLotSegmenter.process(img, cropImage)
			self.parkLotDetected  = self.classifier.classify(self.parkLotSegmenter)
		if exeNature:
			self.natureSegmenter.process(img, cropImage)
			self.natureDetected  = self.classifier.classify(self.natureSegmenter)
		if exeNature and exeParkLot:
			self.occupiedDetected = (not self.natureDetected) and (not self.parkLotDetected)
	def publish(self):
		self.topicInterface.parkLotDetectedPub.publish(self.getParkLotDet()) #always publish this, as a default transition depends on this
		self.topicInterface.visionAppHealth.publish(self.health)
	def getParkLotDet(self):
		return self.parkLotDetected
	def getHealth(self):
		return self.health
	
class VisionAppPC:
	def __init__(self) -> None:
		self.parkLotBounds = ParkingLotSegmenterPC()
	def process(self, img):
		self.parkLotBounds.process(img)
		#self.maxContour = self.parkLotBounds.getContour()
	def getContours(self):
		return self.parkLotBounds.getContours()
	def isPixelsInbound(self, px, py):
		return self.parkLotBounds.isPixelsInbound(px,py)

if __name__ == '__main__':
	visionAppPC = VisionAppPC()
	path = r"C:\WORKSPACE\Capstone\DroneCapstone\src\OperatorApp\Maps\test_image_1.png"
	img = cv2.imread(path)
	visionAppPC.process(img)
	#visionAppPC.parkLotBounds.debugShow()
