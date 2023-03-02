import cv2 as cv
import numpy as np
from pdb import set_trace as bp


if __name__ == '__main__':
	img = cv.imread("ParkingLot01.png")
	img = cv.resize(img, (300*2, 168*2)) 
	img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	ret, threshRGB = cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
	threshHSV = cv.inRange(img_hsv, (0, 0, 0), (180, 30, 255))

	#cv.imshow('rgbThresh', threshRGB)
	cv.imshow('hsvThresh', threshHSV)

	# noise removal
	kernel = np.ones((3,3),np.uint8)
	kernel5 = np.ones((6,6), np.uint8)
	opening = cv.morphologyEx(threshHSV,cv.MORPH_OPEN,kernel5, iterations = 2)
	# sure background area
	sure_bg = cv.dilate(opening,np.ones((12,12), np.uint8), iterations=3)
	# Finding sure foreground area
	#dist_transform = cv.distanceTransform(opening,cv.DIST_L2,5)
	#ret, sure_fg = cv.threshold(dist_transform,0.7*dist_transform.max(),255,0)
	closing = cv.morphologyEx(threshHSV,cv.MORPH_CLOSE,kernel, iterations = 2)
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

	markers = cv.watershed(img,markers)
	img[markers == -1] = [255,0,0]


	cv.imshow("imgShed", img)
	
	# contouring
	#binaryImg = threshHSV;
	binaryImg = np.zeros((img.shape[0], img.shape[1], 1), np.uint8)
	binaryImg[markers == -1] = 255
	contours, hier = cv.findContours(binaryImg, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
	cv.drawContours(binaryImg, contours, -1, 150, 1)
	
	# get largest contour
	maxArea = 0
	maxContour = contours[0]
	for cnt in contours:
		area = cv.contourArea(cnt)
		# debugging for displaying all contours
		#tempImg = np.zeros((img.shape[0], img.shape[1], 1), np.uint8)
		#cv.drawContours(tempImg, [cnt], 0, 255, 3)
		#cv.imshow("currentContour", tempImg)
		#cv.waitKey(0)
		if area == ( (binaryImg.shape[0]-1) * (binaryImg.shape[1]-1) ):
			continue
		# todo
		# must determine if contour is around parking lot or non-parking lot
		# use threshold image with contour bounds and check if sum of threshold image pixels > percentage area of contour?
		if area > maxArea:
			maxArea = area
			maxContour = cnt

	cv.drawContours(binaryImg, [maxContour], 0, 255, 3) 
	cv.imshow("maxContour", binaryImg)
	cv.drawContours(img, [maxContour], 0, (0, 255, 0), 3)
	cv.imshow("parkinglotSeg", img)
	
	cv.waitKey(0)
	
	
	# check if point within contour
	pt = (100, 100)
	# returns 1 if inside, 0 if on contour, -1 if outside
	result = cv.pointPolygonTest(maxContour, pt, False)
	#bp()
	