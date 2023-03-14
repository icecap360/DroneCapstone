import cv2 as cv
import numpy as np
from pdb import set_trace as bp

# debug levels (0 to 2)
debug = 1

def thresholdGray(img, maxHue, maxSaturation):
	img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	#ret, threshRGB = cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
	threshHSV = cv.inRange(img_hsv, (0, 0, 0), (maxHue, maxSaturation, 255))
	
	return threshHSV
	
def applyWatershed(origImg, threshImg):
	# noise removal
	kernel = np.ones((3,3),np.uint8)
	kernel5 = np.ones((6,6), np.uint8)
	opening = cv.morphologyEx(threshImg,cv.MORPH_OPEN,kernel5, iterations = 2)
	# sure background area
	sure_bg = cv.dilate(opening,np.ones((12,12), np.uint8), iterations=3)
	# Finding sure foreground area
	#dist_transform = cv.distanceTransform(opening,cv.DIST_L2,5)
	#ret, sure_fg = cv.threshold(dist_transform,0.7*dist_transform.max(),255,0)
	closing = cv.morphologyEx(threshImg,cv.MORPH_CLOSE,kernel, iterations = 2)
	sure_fg = cv.erode(closing,kernel5,iterations=3)
	# Finding unknown region
	sure_fg = np.uint8(sure_fg)
	unknown = cv.subtract(sure_bg,sure_fg)

	#cv.imshow('bg', sure_bg)
	#cv.imshow('fg', sure_fg)

	# Marker labelling
	ret, markers = cv.connectedComponents(sure_fg)
	# Add one to all labels so that sure background is not 0, but 1
	markers = markers+1
	# Now, mark the region of unknown with zero
	markers[unknown==255] = 0
	markers = cv.watershed(origImg, markers)
	origImg[markers == -1] = [255,0,0]
	
	return img, markers
	
def calcWhiteAreaInContour(contour, threshImg):
	# check all pixels if inside contour
	# if inside, check corresponding pixel on original thresholded image
	# find combined area of white blobs (parking lot) within original thresholded image, bounded by contour
	whiteArea = 0
	for i in range(0, threshImg.shape[0]):
		for j in range(0, threshImg.shape[1]):
			# (x,y)
			pt = (j, i)
			# returns 1 if inside, 0 if on contour, -1 if outside
			within = cv.pointPolygonTest(contour, pt, False)
			if within == 0 or within == 1:
				whiteArea += threshImg[i,j]/255

	# calc percetange area of contour that is white (parking lot)
	return whiteArea / cv.contourArea(contour)
	
def findLargestParkingContour(markers, threshImg, areaThreshold):
	# contouring
	#binaryImg = threshHSV;
	binaryImg = np.zeros((threshImg.shape[0], threshImg.shape[1], 1), np.uint8)
	binaryImg[markers == -1] = 255
	contours, hier = cv.findContours(binaryImg, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
	cv.drawContours(binaryImg, contours, -1, 150, 1)
	
	# get largest contour
	maxArea = 0
	maxContour = contours[0]
	for cnt in contours:
		area = cv.contourArea(cnt)
		#if area == 11410:
		#	bp()
		if area == ( (binaryImg.shape[0]-1) * (binaryImg.shape[1]-1) ):
			# contour is around the whole image
			continue
		# checks if contour is surrounding parking lot area or non-parking lot area
		if area > maxArea:
			percentageArea = calcWhiteAreaInContour(cnt, threshImg)
			if debug == 2:
				print("Percentage area = " + str(percentageArea))
			if (percentageArea >= areaThreshold):
				maxArea = area
				maxContour = cnt
		# debugging for displaying all contours
		if debug == 2:
			tempImg = np.zeros((threshImg.shape[0], threshImg.shape[1], 1), np.uint8)
			cv.drawContours(tempImg, [cnt], 0, 255, 3)
			cv.imshow("currentContour", tempImg)
			cv.waitKey(0)
			
	if debug:
		cv.drawContours(binaryImg, [maxContour], 0, 255, 3) 
		cv.imshow("maxContour", binaryImg)
			
	return maxContour


if __name__ == '__main__':

	img = cv.imread("ParkingLot01.png")
	img = cv.resize(img, (300*2, 168*2)) 
	
	maxHue = 180
	maxSaturation = 30
	areaThreshold = 0.6
	
	# thresholds by HSV values to extract gray regions
	threshHSV = thresholdGray(img, maxHue, maxSaturation)
	
	if debug:
		cv.imshow('hsvThresh', threshHSV)

	# applies watershed segmentation to get boundaries
	img, markers = applyWatershed(img, threshHSV)

	if debug:
		cv.imshow("imgShed", img)
			
	# finds the largest contour of the parking lot
	maxContour = findLargestParkingContour(markers, threshHSV, areaThreshold)

	if debug:
		threshHSV_rgb = cv.cvtColor(threshHSV, cv.COLOR_GRAY2BGR)
		cv.drawContours(threshHSV_rgb, [maxContour], 0, (0, 255, 0), 3)
		cv.imshow("hsvThresh", threshHSV_rgb);
		
	# output display
	cv.drawContours(img, [maxContour], 0, (0, 255, 0), 3)
	cv.imshow("parkinglotSeg", img)
	
	cv.waitKey(0)
	