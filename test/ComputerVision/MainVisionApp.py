import os
from DataSetManager import DataSetManager
from AccuracyMetric import *
from VisionApp import VisionAppPi
import numpy as np
import cv2

def extractWeather(fname:str):
     fname = fname.lower()
     if 'sunny' in fname:
          return 'Sunny'
     return 'Overcast'
def extractClass(fname:str):
    fname = fname.lower()
    if 'nature' in fname:
        return 'Nature'
    if 'parklot' in fname:
        return 'ParkLot'
    return 'Occupied'

if __name__ == "__main__":
    analyzeClassification = False
    analyzeSegments = True
    datasetManager = DataSetManager(imgFolder='SattaliteImagery', annotationFolder='AnnotationSattaliteImagery')
    visionApp = VisionAppPi()

    if analyzeClassification:
        with open('ClassifierResult.csv', 'w') as res:
                res.write('Image Name,Weather,Ground Truth,PredPRK,PredNature,PredOccupied,Accuracy\n')
    if analyzeSegments:
        with open('SegmenterAccuracy.csv', 'w') as res:
            res.write('Image Name, Type, Acc, Prec, Rec \n')
    datasetManager.init()

    for pair in datasetManager.data:
        visionApp.processImage(pair.getImage(),False,True,True)

        if analyzeClassification:
            with open('ClassifierResult.csv', 'a') as res:
                gt = extractClass(pair.getImageName())
                acc = (gt == 'Nature' and visionApp.natureDetected
                        ) or (gt == 'ParkLot' and visionApp.parkLotDetected
                            ) or (gt == 'Occupied' and visionApp.occupiedDetected)
                res.write(','.join([pair.getImageName(),
                                    extractWeather(pair.getImageName()),
                                    extractClass(pair.getImageName()),
                                    str(visionApp.parkLotDetected), 
                                    str(visionApp.natureDetected), 
                                    str(visionApp.occupiedDetected), 
                                    str(acc)]) + '\n' )
                print(visionApp.parkLotSegmenter.getResult().shape)
                cv2.imshow('yeh', visionApp.parkLotSegmenter.getResult())
                cv2.waitKey(0)

        if analyzeSegments:
            with open('SegmenterAccuracy.csv', 'a') as res:
                if pair.hasParkLotAnnot() and pair.hasNatureAnnot():
                    visionApp.processImage(pair.getImage(),False,True,True)
                    
                    pred = visionApp.parkLotSegmenter.getResult()
                    ground = pair.getParkLotAnnot()
                    acc = calculateAccuracy(pred, ground)
                    prec = calculatePrec(pred, ground)
                    rec = calculateRecall(pred, ground)
                    res.write(','.join([pair.getImageName(),'Park Lot', str(acc), str(prec), str(rec)])+'\n')
                    print(pair.getImageName(),'Park Lot Accuracy:', acc, 'Prec:', prec, 'Rec:', rec, 'Pred:', visionApp.parkLotDetected)
                    # numpy_horizontal_concat = np.concatenate((pred, ground), axis=1)
                    # cv2.imshow('Parking Lot Predictions', numpy_horizontal_concat)
                    # cv2.waitKey(0)
                    #visionApp.parkLotSegmenter.debugHSV(50,600)
                    #visionApp.parkLotSegmenter.debugShow(visionApp.parkLotSegmenter.thresh)
                    
                    pred = visionApp.natureSegmenter.getResult()
                    ground = pair.getNatureAnnot()
                    acc = calculateAccuracy(pred, ground)
                    prec = calculatePrec(pred, ground)
                    rec = calculateRecall(pred, ground)
                    res.write(','.join([pair.getImageName(),'Nature', str(acc), str(prec), str(rec)])+'\n')
                    print(pair.getImageName(),'Nature Accuracy:', acc, 'Prec:', prec, 'Rec:', rec, 'Pred:', visionApp.natureDetected)
                    #numpy_horizontal_concat = np.concatenate((pred, ground), axis=1)
                    # visionApp.parkLotSegmenter.debugHSV(100,300)
                    # cv2.imshow('Nature Predictions', numpy_horizontal_concat)
                    # cv2.waitKey(0)
                elif pair.hasParkLotAnnot():
                    visionApp.processImage(pair.getImage(),False,True,False)

                    pred = visionApp.parkLotSegmenter.getResult()
                    ground = pair.getParkLotAnnot()
                    acc = calculateAccuracy(pred, ground)
                    prec = calculatePrec(pred, ground)
                    rec = calculateRecall(pred, ground)
                    print(pair.getImageName(),'Park Lot Accuracy:', acc, 'Prec:', prec, 'Rec:', rec, 'Pred:', visionApp.parkLotDetected)
                    res.write(','.join([pair.getImageName(),'Park Lot', str(acc), str(prec), str(rec)])+'\n')
                    numpy_horizontal_concat = np.concatenate((pred, ground), axis=1)
                    # visionApp.parkLotSegmenter.debugHSV(100,300)
                    # visionApp.parkLotSegmenter.debugShow(visionApp.parkLotSegmenter.thresh)
                    # visionApp.parkLotSegmenter.debugShow(visionApp.parkLotSegmenter.sureBG)
                    # visionApp.parkLotSegmenter.debugShow(visionApp.parkLotSegmenter.sureFG)
                    # visionApp.parkLotSegmenter.debugShow(visionApp.parkLotSegmenter.getResult())
                    cv2.imshow('ParkLot Predictions', numpy_horizontal_concat)
                    cv2.waitKey(0)
