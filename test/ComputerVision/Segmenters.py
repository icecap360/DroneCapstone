import numpy as np
import cv2
from matplotlib import pyplot as plt
from PIL import Image
from abc import ABC, abstractmethod

class Segmenter(ABC):
	def __init__(self, kernels) -> None:
		self.img = None
		self.thresh = None
		self.sureBG = None
		self.sureFG = None
		self.segmentedImg  = None
		self.kernels = kernels
	def debugShow(self, img):
		cv2.imshow("Debug", img)
		cv2.waitKey(0)
	def debugHSV(self, x,y):
		print('X:',x,'Y:',y,'RGB:', self.img[x][y][:], 'HSV:', self.hsv[x][y][:])
		cv2.drawMarker(self.img, ( y,x), (0,0,255), markerType=cv2.MARKER_CROSS, thickness=5)
		cv2.imshow("Debug", self.img)
		cv2.waitKey(0)
	def crop(self, cropW, cropH):
		center = self.img.shape
		x = int(center[1]/2 - cropW/2)
		y = int(center[0]/2 - cropH/2)
		self.img = self.img[int(y):int(y+cropH), int(x):int(x+cropW)]
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
		self.sureFG = cv2.erode(closing,self.kernels["Erode"],iterations=4)
	def createSureBackground(self):
		#kernel = np.ones((3,3),np.uint8)
		opening = cv2.morphologyEx(self.thresh,cv2.MORPH_OPEN,self.kernels["Open"], iterations = 2)
		# sure background area
		self.sureBG = cv2.dilate(opening,self.kernels["Dilate"],iterations=4)
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

	def process(self, img, cropImage):
		self.img = img
		if cropImage:
			# By cropping the image, one cannot directly compare to the original image
			self.crop(200,200)
		self.preprocessRawImg()
		self.threshold()
		self.createSureBackground()
		self.createSureForeground()
		self.watershed()

		_, self.segmentedImg = cv2.threshold(np.uint8(self.markers), 1, 255, cv2.THRESH_BINARY)
		# plt.hist(self.markers.ravel(),256,[0,256])
		# plt.show()
		# cv2.imshow('temp', np.uint8(self.markers))
		# cv2.waitKey(0)

		# self.segmentedImg = np.zeros(self.img.shape, dtype=np.uint8)
		# ret, m2 = cv2.threshold(self.markers.astype(np.uint8), 0, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
		# contours, _ = cv2.findContours(m2, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
		# for c in contours:
		#     #cv2.drawContours(self.segmentedImg, c, -1, (0, 255, 0), 2)
		#     cv2.fillConvexPoly(self.segmentedImg, points=np.int32(c), color=(255,255,255))
		# cv2.imshow('self.segmentedImg', self.segmentedImg)
		# cv2.waitKey(0)
	def getResult(self):
		return self.segmentedImg

class ParkingLotSegmenter(Segmenter):
	def threshold(self):
		self.hsv = cv2.cvtColor(self.img, cv2.COLOR_RGB2HSV)
		# define range of gray color in HSV
		lower_gray = np.array([0, 0, 40])
		upper_gray = np.array([255, 40, 255])
		# Threshold the HSV image to get only gray colors
		self.thresh = cv2.inRange(self.hsv, lower_gray, upper_gray)


class NatureSegmenter(Segmenter):
	def threshold(self):
		hsv = cv2.cvtColor(self.img, cv2.COLOR_RGB2HSV)
		# define range of gray color in HSV
		lower_nat = np.array([20, 50, 50])
		upper_nat = np.array([100, 255, 255])
		# Threshold the HSV image to get only brown,yellow,green colors
		self.thresh = cv2.inRange(hsv, lower_nat, upper_nat)

class ParkingLotBounds(Segmenter):
	def __init__(self, kernels, maxSaturation, areaThreshold) -> None:
		super().__init__(kernels)
		self.maxSaturation = maxSaturation
		self.areaThreshold = areaThreshold
		self.max_contour = None
		self.debug = 0

	def threshold(self):
		img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
		threshHSV = cv2.inRange(img_hsv, (0, 0, 0), (180, self.maxSaturation, 255))

		# close original image to remove noise
		closedImg = cv2.morphologyEx(threshHSV, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8), iterations = 3)

		self.thresh = closedImg

	def calcWhiteAreaInContour(self, contour, threshImg):
		# check all pixels if inside contour
		# if inside, check corresponding pixel on original thresholded image
		# find combined area of white blobs (parking lot) within original thresholded image, bounded by contour
		whiteArea = 0
		for i in range(0, threshImg.shape[0]):
			for j in range(0, threshImg.shape[1]):
				# (x,y)
				pt = (j, i)
				# returns 1 if inside, 0 if on contour, -1 if outside
				within = cv2.pointPolygonTest(contour, pt, False)
				if within == 0 or within == 1:
					whiteArea += threshImg[i,j]/255

		# calc percetange area of contour that is white (parking lot)
		return whiteArea / cv2.contourArea(contour)

	def findLargestParkingContour(self, markers, threshImg, areaThreshold):
		# contouring
		#binaryImg = threshHSV;
		binaryImg = np.zeros((threshImg.shape[0], threshImg.shape[1], 1), np.uint8)
		binaryImg[markers == -1] = 255
		contours, hier = cv2.findContours(binaryImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(binaryImg, contours, -1, 150, 1)

		# get largest contour
		maxArea = 0
		maxContour = contours[0]
		for cnt in contours:
			area = cv2.contourArea(cnt)
			#if area == 11410:
			#	bp()
			if area == ( (binaryImg.shape[0]-1) * (binaryImg.shape[1]-1) ):
				# contour is around the whole image
				continue
			# checks if contour is surrounding parking lot area or non-parking lot area
			if area > maxArea:
				percentageArea = self.calcWhiteAreaInContour(cnt, threshImg)
				if self.debug >= 2:
					print("Percentage area = " + str(percentageArea))
				if (percentageArea >= areaThreshold):
					maxArea = area
					maxContour = cnt
			# debugging for displaying all contours
			if self.debug == 3:
				tempImg = np.zeros((threshImg.shape[0], threshImg.shape[1], 1), np.uint8)
				cv2.drawContours(tempImg, [cnt], 0, 255, 3)
				cv2.imshow("currentContour", tempImg)
				cv2.waitKey(0)

		if self.debug:
			cv2.drawContours(binaryImg, [maxContour], 0, 255, 3)
			cv2.imshow("maxContour", binaryImg)

		return maxContour

	def process(self, img):
		self.img = img

		self.preprocessRawImg()
		self.threshold()
		self.createSureBackground()
		self.createSureForeground()
		self.watershed()

		self.maxContour = self.findLargestParkingContour(self.markers, self.thresh, self.areaThreshold)

		if self.debug:
			cv2.drawContours(self.img, [self.maxContour], 0, (0, 255, 0), 3)
			cv2.imshow("parkinglotSeg", self.img)
			cv2.waitKey(0)

	def getContour(self):
		return self.maxContour
