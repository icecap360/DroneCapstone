import numpy as np
import cv2
from matplotlib import pyplot as plt
from PIL import Image
from abc import ABC
from Utils.Segmenters import ParkingLotSegmenterPI, NatureSegmenter, ParkingLotSegmenterPC
from Utils.Classifier import Classifier

class VisionAppPI:
	def __init__(self) -> None:
		self.health = True
		self.parkLotSegmenter = ParkingLotSegmenterPI()
		self.natureSegmenter = NatureSegmenter()
		self.classifier = Classifier()
		self.parkLotDetected = None
		self.natureDetected = None
		self.occupiedDetected = None
		self.imgLatitude = None
		self.imgLongitude = None
	def init(self, droneCamera, topicInterface):
		self.droneCamera = droneCamera
		self.topicInterface = topicInterface
	def process(self):
		ret = self.droneCamera.read()
		if not ret:
			self.health = False
			return
		self.health = True
		self.imgLatitude = self.topicInterface.getGlobalPose().latitude
		self.imgLongitude = self.topicInterface.getGlobalPose().longitude
		self.processImage(self.droneCamera.image)
	def processImage(self, img, cropImage=True):
		self.parkLotSegmenter.process(img, cropImage)
		self.parkLotDetected  = self.classifier.classify(self.parkLotSegmenter)
		self.natureSegmenter.process(img, cropImage)
		self.natureDetected  = self.classifier.classify(self.natureSegmenter)
		self.occupiedDetected = (not self.natureDetected) and (not self.parkLotDetected)
	def publish(self):
		self.topicInterface.parkLotDetectedPub.publish(self.getParkLotDet()) #always publish this, as a default transition depends on this
		self.topicInterface.visionAppHealth.publish(self.health)
	def getParkLotDet(self):
		return self.parkLotDetected
	def getNatureDet(self):
		return self.natureDetected
	def getOccupiedDet(self):
		return self.occupiedDetected
	def getImgLat(self):
		return self.imgLatitude
	def getImgLong(self):
		return self.imgLongitude
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
