# Author: Winnie
# Date: March 2023
# Purpose: Implements vision app module for PC and PI.

import numpy as np
import cv2
from matplotlib import pyplot as plt
from PIL import Image
from abc import ABC
from Utils.Segmenters import ParkingLotSegmenterPI, NatureSegmenter, ParkingLotSegmenterPC
from Utils.Classifier import Classifier

class VisionAppPI:
	# this object is run on the Raspberry Pi to classify 
	# the center pixel of the camera image as occupied or not
	
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
		# read the latitude, longitude and camera image in quick succession 
		# to ensure values are synchronized
		ret = self.droneCamera.read()
		self.imgLatitude = self.topicInterface.getGlobalPose().latitude
		self.imgLongitude = self.topicInterface.getGlobalPose().longitude
		
		if not ret:
			self.health = False
			return
		self.health = True
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
	# this module is run on the Operator's PC to draw parking lot contours and 
	# warn the user when the desired location is outside a parking lot
	def __init__(self) -> None:
		self.parkLotBounds = ParkingLotSegmenterPC()
	def process(self, img):
		self.parkLotBounds.process(img)
	def getContours(self):
		return self.parkLotBounds.getContours()
	def isPixelsInbound(self, px, py):
		return self.parkLotBounds.isPixelsInbound(px,py)


if __name__ == '__main__':
	visionAppPC = VisionAppPC()
	path = r"C:\WORKSPACE\Capstone\DroneCapstone\src\OperatorApp\Maps\test_image_1.png"
	img = cv2.imread(path)
	visionAppPC.process(img)
