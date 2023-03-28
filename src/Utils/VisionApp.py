import numpy as np
import cv2
from matplotlib import pyplot as plt
from PIL import Image
from abc import ABC
from Utils.Segmenters import ParkingLotSegmenterPI, NatureSegmenter, ParkingLotSegmenterPC
from Utils.Classifier import Classifier

class VisionAppPI:
	def __init__(self) -> None:
		kernels = {
			"Open": np.ones((3,3), np.uint8),
			"Close": np.ones((3,3), np.uint8),
			"Dilate": np.ones((3,3), np.uint8),
			"Erode": np.ones((3,3), np.uint8)
		}
		self.parkLotSegmenter = ParkingLotSegmenterPI(kernels)
		self.natureSegmenter = NatureSegmenter(kernels)
		self.classifier = Classifier()
		self.parkLotDetected = None
		self.natureDetected = None
		self.occupiedDetected = None
	def process(self, img, cropImage=True, exeParkLot=True, exeNature=True):
		if exeParkLot:
			self.parkLotSegmenter.process(img, cropImage)
			self.parkLotDetected  = self.classifier.classify(self.parkLotSegmenter.getResult())
		if exeNature:
			self.natureSegmenter.process(img, cropImage)
			self.natureDetected  = self.classifier.classify(self.natureSegmenter.getResult())
		if exeNature and exeParkLot:
			self.occupiedDetected = (not self.natureDetected) and (not self.parkLotDetected)


class VisionAppPC:
	def __init__(self) -> None:
		kernels = {
			"Open": np.ones((6,6), np.uint8),
			"Close": np.ones((6,6), np.uint8),
			"Dilate": np.ones((12,12), np.uint8),
			"Erode": np.ones((9,9), np.uint8)
		}
		self.parkLotBounds = ParkingLotSegmenterPC(kernels,  areaThreshold=0.6, maxSaturation=35,)
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
