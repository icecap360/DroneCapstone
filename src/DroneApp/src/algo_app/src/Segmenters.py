import numpy as np
import cv2
from matplotlib import pyplot as plt
from PIL import Image
from abc import ABC, abstractmethod

class Segmenter(ABC):
    def __init__(self) -> None:
        self.img = None
        self.thresh = None
        self.sureBG = None
        self.sureFG = None
        self.segmentedImg  = None
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
        kernel = np.ones((3,3),np.uint8)
        closing = cv2.morphologyEx(self.thresh,cv2.MORPH_CLOSE,kernel, iterations = 2)
        # Finding sure foreground area
        #dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        #ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
        self.sureFG = cv2.erode(closing,kernel,iterations=3)
    def createSureBackground(self):
        kernel = np.ones((3,3),np.uint8)
        opening = cv2.morphologyEx(self.thresh,cv2.MORPH_OPEN,kernel, iterations = 2)
        # sure background area
        self.sureBG = cv2.dilate(opening,kernel,iterations=3)
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
