import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import json
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

class DataPair:
    def __init__(self, imgPath):
        self.imgPath = imgPath
        self.parkLotAnnot = None
        self.natureAnnot = None
    def getImage(self):
        return cv2.imread(self.imgPath)
    def hasParkLotAnnot(self):
        return self.parkLotAnnot != None
    def hasNatureAnnot(self):
        return self.natureAnnot != None
    def saveAnnotatedImage(self, folder,polygons, suffix):
        dimensions = cv2.imread(self.imgPath).shape
        img = np.zeros((dimensions[0],dimensions[1]), dtype=np.uint8)
        # for i in range(400):
        #     print(i)
        #     for j in range(640):
        #         for polygon in polygons:
        #             if Polygon(polygon).contains(Point(j,i)):
        #                 img[i][j]=255
        #                 break
        
        for polygon in polygons:
            cv2.fillPoly(img, pts=np.int32([polygon]), color=(255))

        #img = 255-img

        fname = os.path.join(folder, os.path.basename(self.imgPath)[:-4]+'-'+suffix+'.jpg')
        cv2.imwrite(fname, img, [cv2.IMWRITE_JPEG_QUALITY, 100])
        return True

class AnnotProcessor:
    def __init__(self, imgFolder='PublicDataset',annotationFolder= 'AnnotationPublicDataset', annotationFileName='labels_parking-lot-hawk.json') -> None:
        self.annotationFolder = annotationFolder
        annotationFile = os.path.join(self.annotationFolder, annotationFileName)
        with open(annotationFile, 'r') as file:
            self.annotations = json.load(file)
        self.imgFolder = imgFolder
        self.data = []
    def init(self):
        dataFiles = np.sort(os.listdir(self.imgFolder))
        for imgFile in dataFiles:
            if not '.jpg' in imgFile and not '.png' in imgFile:
                continue
            #coreImageName = os.path.basename(imgFile).split('-')[:-1].join('-')
            fullImgPath = os.path.join(self.imgFolder, imgFile) 
            dataPair = DataPair(fullImgPath)
            if imgFile in self.annotations.keys():
                annots = self.extractAnnotations(self.annotations[imgFile])
                if len(annots[0])>0:
                    dataPair.parkLotAnnot = annots[0]
                if len(annots[1])>0:
                    dataPair.natureAnnot = annots[1]
                self.data.append(dataPair)
            else:
                self.data.append(dataPair)
    def extractAnnotations(self, annotDict):
        parkLots, nature = [],[]
        annots = annotDict['regions']
        for i in range(len(annots)):
            points = np.column_stack((annots[str(i)]["shape_attributes"]["all_points_x"], annots[str(i)]["shape_attributes"]["all_points_y"]))
            if annots[str(i)]["region_attributes"]["label"] == "ParkingLot":
                parkLots.append(points)
            elif annots[str(i)]["region_attributes"]["label"] == "Nature":
                nature.append(points)
        return parkLots, nature
    
    def updateAnnotatedImages(self):
        for pair in self.data:
            if pair.hasParkLotAnnot():
                print('Processing Parking Lot Annotation ', pair.imgPath, '...')
                pair.saveAnnotatedImage(self.annotationFolder,pair.parkLotAnnot,  'AnnotPRK')
            if pair.hasNatureAnnot():
                print('Processing Nature Annotation', pair.imgPath, '...')
                pair.saveAnnotatedImage(self.annotationFolder,pair.natureAnnot,  'AnnotNAT')

if __name__=='__main__':
    dataset = AnnotProcessor(imgFolder='SattaliteImagery', annotationFolder='AnnotationSattaliteImagery',annotationFileName='labels_ali-satellite-imagery_2023-03-08-10-35-12.json')#labels_prk+nat_2023-03-04-11-58-06.json')
    dataset.init()
    dataset.updateAnnotatedImages() 

    # dataset = AnnotProcessor(annotationFileName='labels_parking-lot-hawk.json')#labels_prk+nat_2023-03-04-11-58-06.json')
    # dataset.init()
    # dataset.updateAnnotatedImages() 

    # dataset = AnnotProcessor(annotationFileName='labels_prk+nat_2023-03-04-11-58-06.json')
    # dataset.init()
    # dataset.updateAnnotatedImages()

        