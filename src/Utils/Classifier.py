import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter

class Classifier:
    def __init__(self) -> None:
        self.threshold = 0.5
        self.filterSz = 17
        self.sigma = 4
        temp = np.zeros((self.filterSz,self.filterSz))
        temp[self.filterSz//2][self.filterSz//2] = 1
        #self.filter  =cv2.GaussianBlur(temp, (self.filterSz,self.filterSz),self.sigma)
        self.filter  =gaussian_filter(temp, self.sigma)
    def plotFilter(self):
        fig, ax = plt.subplots()
        im = ax.imshow(filter)
        for i in range(self.filterSz):
            for j in range(self.filterSz):
                text = ax.text(j, i, np.round(filter[i, j],3),
                            ha="center", va="center", color="w", fontsize=5)
        print(filter)
        plt.show()
    def classify(self, segmenter, name='' ):
        centerX, centerY = segmenter.getResult().shape[0]//2, segmenter.getResult().shape[1]//2
        halfFilSz = self.filterSz//2
        centerSquare = np.zeros((self.filterSz, self.filterSz))
        for i in range(self.filterSz):
            for j in range(self.filterSz):
                centerSquare[i][j] = segmenter.isPixelsInbound(centerX-halfFilSz+i,centerY-halfFilSz+j)
        #centerSquare = segmenter.getResult()[(centerX-halfFilSz):(centerX+halfFilSz+1),(centerY-halfFilSz):(centerY+halfFilSz+1)]
        #binCenterSq = centerSquare>0 #turn into boolean
        prob = np.sum(np.multiply(centerSquare,self.filter))
        #print(name+'Prob:', prob)
        return prob>self.threshold
        

if __name__ =='__main__':
    pass
    # img = cv2.imread(r"C:\WORKSPACE\Capstone\ComputerVision\PublicDataset\1Drone-Sunny-MedAlt-Nature.jpg")
    # print(img.shape)
    # centerX, centerY = img.shape[0]//2, img.shape[1]//2
    # halfFilSz = 17//2
    # centerSquare = img[(centerX-halfFilSz):(centerX+halfFilSz+1),(centerY-halfFilSz):(centerY+halfFilSz+1),:]
    # print(centerSquare.shape,centerX-halfFilSz,centerX+halfFilSz)
    # cv2.imshow('gfd', centerSquare)
    # cv2.waitKey(0)
