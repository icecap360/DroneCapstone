import numpy as np
import cv2
from matplotlib import pyplot as plt
from PIL import Image

def calculateAccuracy( pred, ground):
        #return np.mean(np.abs(np.subtract(pred, ground)))
        # length, width = pred.shape[0], pred.shape[1]
        # #plt.hist(pred.ravel(),256,[0,256]); plt.show()
        # diff = np.subtract(pred.reshape((length,width)),ground.reshape((length,width)))
        # _, binarized = cv2.threshold(np.uint8(diff), 125, 255, cv2.THRESH_BINARY) #threshold at 125, the middle
        # #plt.hist(binarized.ravel(),256,[0,256]); plt.show()
        # acc = np.mean(binarized<255)
        acc = np.mean(pred==ground)
        return acc

def calculatePrec( pred, ground):
        tp = np.sum(np.logical_and(pred==255, ground==255))/np.sum(ground==255)
        fp = np.sum(np.logical_and(pred==255, ground==0))/np.sum(ground==0)
        return tp/(tp+fp)
def calculateRecall( pred, ground):
        tp = np.sum(np.logical_and(pred==255, ground==255))/np.sum(ground==255)
        fn = np.sum(np.logical_and(pred==0, ground==255))/np.sum(ground==255)
        return tp/(tp+fn)