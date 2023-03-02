import requests

def get_street_image(latitude, longitude):
    API_URL = f'https://static-maps.yandex.ru/1.x/?ll={longitude},{latitude}&z=19&l=sat&size=600,400'
    #z is for zoom here and the last 2 numbers are for image pixel dimensions
    
    response = requests.get(API_URL)
    
    if response.status_code == 200:
        with open("street_image.png", "wb") as f:
            f.write(response.content)
        return True
    else:
        return False
get_street_image(43.263028258992236, -79.91665084063234)


#Watershed Algo

import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
from PIL import Image


img = cv.imread('street_image.png', cv.IMREAD_ANYCOLOR)
# Convert RGB to HSV
hsv = cv.cvtColor(img, cv.COLOR_RGB2HSV)
# define range of gray color in HSV
lower_gray = np.array([0, 0, 0])
upper_gray = np.array([255, 25, 255])
# Threshold the HSV image to get only gray colors
thresh = cv.inRange(hsv, lower_gray, upper_gray)
cv.imshow("thresholding", thresh)



#gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
#ret, thresh = cv.threshold(gray,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
#print(ret)

# noise removal
kernel = np.ones((3,3),np.uint8)
opening = cv.morphologyEx(thresh,cv.MORPH_OPEN,kernel, iterations = 2)
# sure background area
sure_bg = cv.dilate(opening,kernel,iterations=3)
closing = cv.morphologyEx(thresh,cv.MORPH_CLOSE,kernel, iterations = 2)
# Finding sure foreground area
#dist_transform = cv.distanceTransform(opening,cv.DIST_L2,5)
#ret, sure_fg = cv.threshold(dist_transform,0.7*dist_transform.max(),255,0)
sure_fg = cv.erode(closing,kernel,iterations=3)

# Finding unknown region
sure_fg = np.uint8(sure_fg)
unknown = cv.subtract(sure_bg,sure_fg)

# Marker labelling
ret, markers = cv.connectedComponents(sure_fg)
# Add one to all labels so that sure background is not 0, but 1
markers = markers+1
# Now, mark the region of unknown with zero
markers[unknown==255] = 0

markers = cv.watershed(img,markers)
img[markers == -1] = [255,0,0]


cv.imshow("pic", img)



#Contouring

import cv2
import numpy as np

# Let's load a simple image with 3 black squares
img2 = np.ones((400, 600, 3), dtype = np.uint8)
img2[markers == -1] = [255,0,0]
image = img2


# Grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Find Canny edges
edged = cv2.Canny(gray, 30, 200)


# Finding Contours
# Use a copy of the image e.g. edged.copy()
# since findContours alters the image
contours, hierarchy = cv2.findContours(edged,
	cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

cv2.imshow('Canny Edges After Contouring', edged)


print("Number of Contours found = " + str(len(contours)))

# Draw all contours
# -1 signifies drawing all contours
cv2.drawContours(image, contours, -1, (0, 255, 0), 1)

cv2.imshow('Contours', image)
cv2.imwrite('Contoured.png', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
