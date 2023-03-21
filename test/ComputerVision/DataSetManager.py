import cv2
import os
import json
import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

class DataPair:
    def __init__(self, imgPath):
        self.imgPath = imgPath
        self.parkLotAnnotPath = None
        self.natureAnnotPath = None
    def getImage(self):
        return cv2.imread(self.imgPath)
    def getParkLotAnnot(self):
        img = cv2.imread(self.parkLotAnnotPath, cv2.IMREAD_GRAYSCALE)
        _, img = cv2.threshold(np.uint8(img), 125, 255, cv2.THRESH_BINARY) #threshold at 125, the middle
        return img
    def getNatureAnnot(self):
        img = cv2.imread(self.natureAnnotPath, cv2.IMREAD_GRAYSCALE)
        _, img = cv2.threshold(np.uint8(img), 125, 255, cv2.THRESH_BINARY) #threshold at 125, the middle
        return img
    def hasNatureAnnot(self):
        return self.natureAnnotPath != None
    def getImageName(self):
        return os.path.basename(self.imgPath)[:-4]
    def hasParkLotAnnot(self):
        return self.parkLotAnnotPath != None

class DataSetManager:
    def __init__(self, imgFolder='PublicDataset',annotationFolder= 'AnnotationPublicDataset') -> None:
        self.annotationFolder = annotationFolder
        self.imgFolder = imgFolder
        self.data = []
    def init(self):
        imgFiles = np.sort(os.listdir(self.imgFolder))
        annotFiles = np.sort(os.listdir(self.annotationFolder))
        for imgFile in imgFiles:
            if not '.jpg' in imgFile and not '.png' in imgFile:
                continue
            parkLotAnnotPath = os.path.basename(imgFile)[:-4]+'-AnnotPRK.jpg'
            fullParkAnnotPath = os.path.join(self.annotationFolder, parkLotAnnotPath)
            natureAnnotPath = os.path.basename(imgFile)[:-4]+'-AnnotNAT.jpg'
            fullNatureAnnotPath = os.path.join(self.annotationFolder, natureAnnotPath)
            fullImgPath = os.path.join(self.imgFolder, imgFile) 
            dataPair = DataPair(fullImgPath)
            if parkLotAnnotPath in annotFiles:
                dataPair.parkLotAnnotPath = fullParkAnnotPath
            if natureAnnotPath in annotFiles:
                dataPair.natureAnnotPath = fullNatureAnnotPath
            self.data.append(dataPair)

