import numpy as np
import cv2
from matplotlib import pyplot as plt
from PIL import Image
from abc import ABC
from Segmenters import ParkingLotSegmenter, NatureSegmenter, ParkingLotBounds


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
