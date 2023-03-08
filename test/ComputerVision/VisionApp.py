import numpy as np
import cv2 
from matplotlib import pyplot as plt
from PIL import Image
from abc import ABC
from Segmenters import ParkingLotSegmenter, NatureSegmenter
from DataSetManager import DataSetManager
from Classifier import Classifier
class VisionAppPi:
    def __init__(self) -> None:
        self.parkLotSegmenter = ParkingLotSegmenter()
        self.natureSegmenter = NatureSegmenter()
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
