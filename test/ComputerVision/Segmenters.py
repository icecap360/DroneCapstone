import numpy as np
import cv2
from matplotlib import pyplot as plt
from PIL import Image
from abc import ABC, abstractmethod
import copy
class Segmenter(ABC):
	def __init__(self, kernels, areaThreshold, iterations) -> None:
		self.img = None
		self.thresh = None
		self.sureBG = None
		self.sureFG = None
		self.segmentedImg  = None
		self.kernels = kernels
		self.iterations = iterations
		self.areaThreshold = areaThreshold
	def debugShow(self, img):
		cv2.imshow("Debug", img)
		cv2.waitKey(0)
	def debugHSV(self, x,y):
		print('X:',x,'Y:',y,'RGB:', self.img[x][y][:], 'HSV:', self.hsv[x][y][:])
		cv2.drawMarker(self.img, ( y,x), (0,0,255), markerType=cv2.MARKER_CROSS, thickness=5)
		cv2.imshow("Debug", self.img)
		cv2.waitKey(0)
	def crop(self, cropW, cropH):
		x_min = int(self.img.shape[1]/2 - cropW/2)
		y_min = int(self.img.shape[0]/2 - cropH/2)
		self.img = self.img[int(y_min):int(y_min+cropH), int(x_min):int(x_min+cropW)]
	def preprocessRawImg(self):
		hsv = cv2.cvtColor(self.img, cv2.COLOR_RGB2HSV)
		hsv[:][:][2] = cv2.equalizeHist(hsv[:][:][2])
		self.img = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)


	@abstractmethod
	def threshold(self):
		pass
	def createSureForeground(self):
		#kernel = np.ones((3,3),np.uint8)
		closing = cv2.morphologyEx(self.thresh,cv2.MORPH_CLOSE,self.kernels["Close"], iterations = 2)
		# Finding sure foreground area
		#dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
		#ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
		self.sureFG = cv2.erode(closing,self.kernels["Erode"],iterations=self.iterations)
	def createSureBackground(self):
		#kernel = np.ones((3,3),np.uint8)
		opening = cv2.morphologyEx(self.thresh,cv2.MORPH_OPEN,self.kernels["Open"], iterations = 2)
		# sure background area
		self.sureBG = cv2.dilate(opening,self.kernels["Dilate"],iterations=self.iterations)
	def watershed(self):
		# Finding unknown region
		self.sureFG = np.uint8(self.sureFG)
		unknown = cv2.subtract(self.sureBG,self.sureFG)

		# Marker labelling
		ret, markers = cv2.connectedComponents(self.sureFG)

		# Add one to all labels so that sure background is not 0, but 1
		markers = markers+1
		# Now, mark the region of unknown with zero
		markers[unknown==255] = 0

		self.markers = cv2.watershed(self.img,markers)
		#self.img[markers == -1] = [255,0,0]

	def findGroups(self):
		self.groups = []
		for group in range(np.min(self.markers), np.max(self.markers)+1):
			area = np.equal(self.markers, group)
			if np.sum(area) == 0:
				continue
			areaOfInterest = np.logical_and( area, self.thresh)
			percInterest = np.sum(areaOfInterest) / np.sum(area)
			if percInterest > self.areaThreshold:
				self.groups.append(group)
	def isPixelsInbound(self, px, py):
		return self.segmentedImg[int(px)][int(py)][0]>0

	def getSegmentation(self):
		self.segmentedImg = np.zeros((self.thresh.shape[0], self.thresh.shape[1], 1), np.uint8)
		for group in self.groups:
			self.segmentedImg[self.markers == group] = 255
	
	def findContours(self):
		self.contours, _ = cv2.findContours(self.segmentedImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	def process(self, img, cropImage=False, preprocess=False, findContours=False):
		self.img = img
		if cropImage:
			# By cropping the image, one cannot directly compare to the original image
			self.crop(200,200)
		if preprocess:
			self.preprocessRawImg()
		self.threshold()
		self.createSureBackground()
		self.createSureForeground()
		self.watershed()
		self.findGroups()
		self.getSegmentation()
		if findContours:
			self.findContours()
			

	def getResult(self):
		return self.segmentedImg
	def getContours(self):
		return self.contours

class ParkingLotSegmenterPI(Segmenter):
	def threshold(self):
		self.hsv = cv2.cvtColor(self.img, cv2.COLOR_RGB2HSV)
		# define range of gray color in HSV
		lower_gray = np.array([0, 0, 40])
		upper_gray = np.array([255, 80, 160])
		# Threshold the HSV image to get only gray colors
		self.thresh = cv2.inRange(self.hsv, lower_gray, upper_gray)
	def __init__(self):
		kernels = {
			"Open": np.ones((3,3), np.uint8),
			"Close": np.ones((3,3), np.uint8),
			"Dilate": np.ones((3,3), np.uint8),
			"Erode": np.ones((3,3), np.uint8)
		}
		super().__init__(kernels, areaThreshold=0.6, iterations=3)


class NatureSegmenter(Segmenter):
	def threshold(self):
		self.hsv = cv2.cvtColor(self.img, cv2.COLOR_RGB2HSV)
		# define range of gray color in HSV
		lower_nat = np.array([20, 80, 40])
		upper_nat = np.array([100, 255, 255])
		# Threshold the HSV image to get only brown,yellow,green colors
		self.thresh = cv2.inRange(self.hsv, lower_nat, upper_nat)
	def __init__(self):
		kernels = {
			"Open": np.ones((3,3), np.uint8),
			"Close": np.ones((3,3), np.uint8),
			"Dilate": np.ones((3,3), np.uint8),
			"Erode": np.ones((3,3), np.uint8)
		}
		super().__init__(kernels, areaThreshold=0.6, iterations=2)

class ParkingLotSegmenterPC(Segmenter):
	def __init__(self) -> None:
		kernels = {
			"Open": np.ones((6,6), np.uint8),
			"Close": np.ones((6,6), np.uint8),
			"Dilate": np.ones((12,12), np.uint8),
			"Erode": np.ones((9,9), np.uint8)
		}
		super().__init__(kernels, areaThreshold=0.6,iterations=4)
		self.maxSaturation = 18
		self.max_contour = None
		self.debug = 0

	def threshold(self):
		self.hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
		threshHSV = cv2.inRange(self.hsv, (0, 0, 40), (180, self.maxSaturation, 255))
		# close original image to remove noise
		closedImg = cv2.morphologyEx(threshHSV, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8), iterations = 3)

		self.thresh = closedImg

	def process(self, img):
		super().process(img, findContours=True)
		if self.debug:
			plt.imshow(self.segmentedImg)
			# plt.imsave('segmentedImg.png', np.stack((self.segmentedImg, 
			# 		   np.zeros(self.segmentedImg.shape), 
			# 		   np.zeros(self.segmentedImg.shape)), axis=2))
			cv2.imwrite('segmentedImg.png',self.segmentedImg)
			plt.show()
			cv2.drawContours(self.img, self.getContours(), -1, (0, 255, 0), 3)
			plt.imshow( self.img)
			plt.show()

