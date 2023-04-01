import os
from DataSetManager import DataSetManager
from AccuracyMetric import *
from VisionApp import VisionAppPI
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
    analyzeClassification = True
    analyzeSegments = True
    datasetManager = DataSetManager(imgFolder='PublicDataset', annotationFolder='AnnotationPublicDataset')
    visionApp = VisionAppPI()

    if analyzeClassification:
        with open('ClassifierResult.csv', 'w') as res:
                res.write('Image Name,Weather,Ground Truth,PredPRK,PredNature,PredOccupied,Accuracy\n')
    if analyzeSegments:
        with open('SegmenterAccuracy.csv', 'w') as res:
            res.write('Image Name, Type, Acc, Prec, Rec \n')
    datasetManager.init()

    correct, total = 0,0
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

                # visionApp.parkLotSegmenter.debugHSV(200,320)

                if acc:
                    correct +=1
                total+=1
                #cv2.imshow('yeh', visionApp.parkLotSegmenter.getResult())
                #cv2.waitKey(0)

        if analyzeSegments:
            with open('SegmenterAccuracy.csv', 'a') as res:
                if pair.hasParkLotAnnot() and pair.hasNatureAnnot():
                    visionApp.processImage(pair.getImage(),False,True,True)
                    
                    pred = visionApp.parkLotSegmenter.getResult()
                    ground = pair.getParkLotAnnot()
                    pred = np.reshape(pred, ground.shape)
                    acc = calculateAccuracy(pred, ground)
                    prec = calculatePrec(pred, ground)
                    rec = calculateRecall(pred, ground)
                    res.write(','.join([pair.getImageName(),'Park Lot', str(acc), str(prec), str(rec)])+'\n')
                    print(pair.getImageName(),'Park Lot Accuracy:', acc, 'Prec:', prec, 'Rec:', rec, 'Pred:', visionApp.parkLotDetected)
                    numpy_horizontal_concat = np.concatenate((pred, ground), axis=1)
                    # cv2.imshow('Parking Lot Predictions', numpy_horizontal_concat)
                    # cv2.imshow('markers', visionApp.parkLotSegmenter.markers.astype(np.uint8))
                    # cv2.imshow('thresh', visionApp.parkLotSegmenter.thresh.astype(np.uint8))
                    # cv2.waitKey(0)
                    # visionApp.parkLotSegmenter.debugHSV(200,320)
                    # visionApp.parkLotSegmenter.debugShow(visionApp.parkLotSegmenter.thresh)
                    
                    pred = visionApp.natureSegmenter.getResult()
                    ground = pair.getNatureAnnot()
                    pred = np.reshape(pred, ground.shape)
                    acc = calculateAccuracy(pred, ground)
                    prec = calculatePrec(pred, ground)
                    rec = calculateRecall(pred, ground)
                    res.write(','.join([pair.getImageName(),'Nature', str(acc), str(prec), str(rec)])+'\n')
                    print(pair.getImageName(),'Nature Accuracy:', acc, 'Prec:', prec, 'Rec:', rec, 'Pred:', visionApp.natureDetected)
                    # numpy_horizontal_concat = np.concatenate((pred, ground), axis=1)
                    #visionApp.natureSegmenter.debugHSV(200,320)
                    # cv2.imshow('Nature Predictions', numpy_horizontal_concat)
                    # cv2.waitKey(0)
                elif pair.hasParkLotAnnot():
                    visionApp.processImage(pair.getImage(),False,True,False)
                    pred = visionApp.parkLotSegmenter.getResult()
                    ground = pair.getParkLotAnnot()
                    pred = np.reshape(pred, ground.shape)
                    acc = calculateAccuracy(pred, ground)
                    prec = calculatePrec(pred, ground)
                    rec = calculateRecall(pred, ground)
                    print(pair.getImageName(),'Park Lot Accuracy:', acc, 'Prec:', prec, 'Rec:', rec, 'Pred:', visionApp.parkLotDetected)
                    res.write(','.join([pair.getImageName(),'Park Lot', str(acc), str(prec), str(rec)])+'\n')
                    numpy_horizontal_concat = np.concatenate((pred, ground), axis=1)
                    # visionApp.parkLotSegmenter.debugHSV(200,320)
                    # visionApp.parkLotSegmenter.debugShow(visionApp.parkLotSegmenter.thresh)
                    # visionApp.parkLotSegmenter.debugShow(visionApp.parkLotSegmenter.sureBG)
                    # visionApp.parkLotSegmenter.debugShow(visionApp.parkLotSegmenter.sureFG)
                    # visionApp.parkLotSegmenter.debugShow(visionApp.parkLotSegmenter.getResult())
                    #cv2.imshow('ParkLot Predictions', numpy_horizontal_concat)
                    #cv2.waitKey(0)
                
    
    
    print('Final Accuracy', correct/total, correct, total)