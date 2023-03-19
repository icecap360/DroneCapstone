import numpy as np
import cv2
from matplotlib import pyplot as plt
from PIL import Image
from abc import ABC
from Segmenters import ParkingLotSegmenter, NatureSegmenter, ParkingLotBounds
from DataSetManager import DataSetManager
from Classifier import Classifier
class VisionAppPi:
	def __init__(self) -> None:

		kernels = {
			"Open": np.ones((3,3), np.uint8),
			"Close": np.ones((3,3), np.uint8),
			"Dilate": np.ones((3,3), np.uint8),
			"Erode": np.ones((3,3), np.uint8)
		}
		self.parkLotSegmenter = ParkingLotSegmenter(kernels)
		self.natureSegmenter = NatureSegmenter(kernels)
		self.classifier = Classifier()
		self.parkLotDetected = None
		self.natureDetected = None
		self.occupiedDetected = None
	def processImage(self, img, cropImage=True, exeParkLot=True, exeNature=True):
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
		self.parkLotBounds = ParkingLotBounds(kernels, maxSaturation=35, areaThreshold=0.6)
		self.maxContour = None

	def processImage(self, img):
		self.parkLotBounds.process(img)
		self.maxContour = self.parkLotBounds.getContour()


# if __name__ == '__main__':
# 	visionAppPC = VisionAppPC()
#
# 	for i in range(0,30):
# 		img = cv2.imread("SattaliteImagery\\TestSet\\ParkingLot" + str(i).zfill(2) + ".png")
# 		visionAppPC.processImage(img)
