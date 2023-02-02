import cv2 as cv
import numpy as np


if __name__ == '__main__':
	img = cv.imread("ParkingLot01.jpg")
	img = cv.resize(img, (300*2, 168*2)) 
	img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	ret, threshRGB = cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
	threshHSV = cv.inRange(img_hsv, (0, 0, 0), (180, 30, 255))

	cv.imshow('rgbThresh', threshRGB)
	cv.imshow('hsvThresh', threshHSV)

	# noise removal
	kernel = np.ones((3,3),np.uint8)
	opening = cv.morphologyEx(threshHSV,cv.MORPH_OPEN,kernel, iterations = 2)
	# sure background area
	sure_bg = cv.dilate(opening,kernel,iterations=3)
	# Finding sure foreground area
	#dist_transform = cv.distanceTransform(opening,cv.DIST_L2,5)
	#ret, sure_fg = cv.threshold(dist_transform,0.7*dist_transform.max(),255,0)
	closing = cv.morphologyEx(threshHSV,cv.MORPH_CLOSE,kernel, iterations = 2)
	sure_fg = cv.erode(closing,kernel,iterations=3)
	# Finding unknown region
	sure_fg = np.uint8(sure_fg)
	unknown = cv.subtract(sure_bg,sure_fg)

	cv.imshow('bg', sure_bg)
	cv.imshow('fg', sure_fg)

	# Marker labelling
	ret, markers = cv.connectedComponents(sure_fg)
	# Add one to all labels so that sure background is not 0, but 1
	markers = markers+1
	# Now, mark the region of unknown with zero
	markers[unknown==255] = 0

	markers = cv.watershed(img,markers)
	img[markers == -1] = [255,0,0]


	cv.imshow("imgShed", img)
	cv.waitKey(0)


