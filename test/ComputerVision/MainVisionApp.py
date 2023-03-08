import os
from DataSetManager import DataSetManager
from AccuracyMetric import *
from VisionApp import VisionAppPi
import numpy as np
import cv2

if __name__ == "__main__":
    datasetManager = DataSetManager()
    visionApp = VisionAppPi()
    with open('ClassifierResult.csv', 'w') as res:
            res.write('Image Name, Ground Truth, PredPRK, PredNature, PredOccupied\n')
    with open('SegmenterAccuracy.csv', 'w') as res:
            res.write('Image Name, Type, Acc, Prec, Rec \n')
    datasetManager.init()

    for pair in datasetManager.data:
        visionApp.processImage(pair.getImage(),False,True,True)

        with open('ClassifierResult.csv', 'a') as res:
            gt = 'ParkLot'
            if 'Nature' in pair.getImageName():
                gt = 'Nature'
            elif 'Occupied' in pair.getImageName():
                gt = 'Occupied'
            res.write(','.join([pair.getImageName(),gt,str(visionApp.parkLotDetected), 
                       str(visionApp.natureDetected), str(visionApp.occupiedDetected)])+'\n' )
            print(visionApp.parkLotSegmenter.getResult().shape)
            cv2.imshow('yeh', visionApp.parkLotSegmenter.getResult())
            cv2.waitKey(0)

        # with open('SegmenterAccuracy.csv', 'a') as res:
        #     if pair.hasParkLotAnnot() and pair.hasNatureAnnot():
        #         visionApp.processImage(pair.getImage(),False,True,True)
                
        #         pred = visionApp.parkLotSegmenter.getResult()
        #         ground = pair.getParkLotAnnot()
        #         acc = calculateAccuracy(pred, ground)
        #         prec = calculatePrec(pred, ground)
        #         rec = calculateRecall(pred, ground)
        #         res.write(','.join([pair.getImageName(),'Park Lot', str(acc), str(prec), str(rec)])+'\n')
        #         print(pair.getImageName(),'Park Lot Accuracy:', acc, 'Prec:', prec, 'Rec:', rec, 'Pred:', visionApp.parkLotDetected)
        #         # numpy_horizontal_concat = np.concatenate((pred, ground), axis=1)
        #         # cv2.imshow('Parking Lot Predictions', numpy_horizontal_concat)
        #         # cv2.waitKey(0)
        #         #visionApp.parkLotSegmenter.debugHSV(50,600)
        #         #visionApp.parkLotSegmenter.debugShow(visionApp.parkLotSegmenter.thresh)
                
        #         pred = visionApp.natureSegmenter.getResult()
        #         ground = pair.getNatureAnnot()
        #         acc = calculateAccuracy(pred, ground)
        #         prec = calculatePrec(pred, ground)
        #         rec = calculateRecall(pred, ground)
        #         res.write(','.join([pair.getImageName(),'Nature', str(acc), str(prec), str(rec)])+'\n')
        #         print(pair.getImageName(),'Nature Accuracy:', acc, 'Prec:', prec, 'Rec:', rec, 'Pred:', visionApp.natureDetected)
        #         #numpy_horizontal_concat = np.concatenate((pred, ground), axis=1)
        #         # visionApp.parkLotSegmenter.debugHSV(100,300)
        #         # cv2.imshow('Nature Predictions', numpy_horizontal_concat)
        #         # cv2.waitKey(0)
        #     elif pair.hasParkLotAnnot():
        #         visionApp.processImage(pair.getImage(),False,True,False)

        #         pred = visionApp.parkLotSegmenter.getResult()
        #         ground = pair.getParkLotAnnot()
        #         acc = calculateAccuracy(pred, ground)
        #         prec = calculatePrec(pred, ground)
        #         rec = calculateRecall(pred, ground)
        #         print(pair.getImageName(),'Park Lot Accuracy:', acc, 'Prec:', prec, 'Rec:', rec, 'Pred:', visionApp.parkLotDetected)
        #         res.write(','.join([pair.getImageName(),'Park Lot', str(acc), str(prec), str(rec)])+'\n')
        #         numpy_horizontal_concat = np.concatenate((pred, ground), axis=1)
        #         numpy_horizontal_concat = np.concatenate((pred, ground), axis=1)
        #         # visionApp.parkLotSegmenter.debugHSV(100,300)
        #         cv2.imshow('Nature Predictions', numpy_horizontal_concat)
        #         cv2.waitKey(0)
