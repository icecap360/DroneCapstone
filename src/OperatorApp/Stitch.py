import requests
from PIL import Image
import os
import math
import urllib.request
from io import BytesIO
import sys
import cv2
from Utils import VisionAppPC
import numpy as np

class StitchCreator:
    def __init__(self, lat, long):
        # Enter your api key here
        self.api_key = 'AIzaSyCDtvdqQzxtHOt9dMR4WQPU50tO-L5K_qU'
        self.url = "https://maps.googleapis.com/maps/api/staticmap?"
        self.center = "Dehradun"
        self.zoom = 19
        self.sz = 640
        self.scale = 2
        self.szStr = "%dx%d" %(self.sz,self.sz)
        self.lat = lat
        self.long = long
        self.longround= 0.003
        self.latround=0.002
    def latfactor(self, poilat):
        return ((self.sz*360) / (512*2**(self.zoom - 1)) * math.cos(poilat))

    def longfactor(self):
        return (self.sz*360) / (512* 2**(self.zoom - 1))
       
    def FindCenter(self):
        poilat = round(self.lat / self.latround) * self.latround
        poilong = round(self.long / self.longround) * self.longround
        return (poilat, poilong)
    def StitchAndSave(self):
        roundlat, roundlong = self.FindCenter()
        poilat, poilong = roundlat, roundlong
        
        positions=[]

        positions.append(str(poilat+self.latfactor(poilat+self.latfactor(poilat))) + ',' + str(poilong-self.longfactor()))
        positions.append(str(poilat+self.latfactor(poilat+self.latfactor(poilat))) + ',' + str(poilong))
        positions.append(str(poilat+self.latfactor(poilat+self.latfactor(poilat))) + ',' + str(poilong+self.longfactor()))
        positions.append(str(poilat) + ',' + str(poilong-self.longfactor()))
        positions.append(str(poilat) + ',' + str(poilong))
        positions.append(str(poilat) + ',' + str(poilong+self.longfactor()))
        positions.append(str(poilat-self.latfactor(poilat-self.latfactor(poilat))) + ',' + str(poilong-self.longfactor()))
        positions.append(str(poilat-self.latfactor(poilat-self.latfactor(poilat))) + ',' + str(poilong))
        positions.append(str(poilat-self.latfactor(poilat-self.latfactor(poilat))) + ',' + str(poilong+self.longfactor()))

        new_image = Image.new('RGB', (self.sz*3, self.sz*3))

        for i in range(9):
            final = Image.new("RGB", (self.sz, self.sz))
            urlparams = urllib.parse.urlencode({'key':self.api_key,
                                                'center': positions[i],
                                                'zoom': str(self.zoom),
                                                'size': '%dx%d' % (self.sz,self.sz),
                                                'maptype': 'satellite',
                                                'sensor': 'false',
                                                'scale': self.scale})
            url = 'http://maps.google.com/maps/api/staticmap?' + urlparams


            # f=urllib.request.urlopen(url)
            # im=Image.open(BytesIO(f.read()))
            # final.paste(im, (0, 0))

            r = requests.get(url + "center =" + self.center + "&zoom =" +
                            str(self.zoom) + "&size = "+self.szStr+"&key =" +
                                        self.api_key + "maptype=satellite&sensor = false")
            im=Image.open(BytesIO(r.content))
            
            final.paste(im,(0,0))
            
            x = (i % 3) * self.sz
            y = (i // 3) * self.sz
                
            # Paste the image onto the new image
            new_image.paste(final, (x, y))

            
        filename=str(roundlat)+"_"+str(roundlong)+"_" + str(self.latfactor(poilat+self.latfactor(poilat))) + "_" +str(self.latfactor(poilat))+"_"+ str(self.latfactor(poilat-self.latfactor(poilat)))+'.png'
        filePath = os.path.abspath(os.path.join('Maps', filename))
        os.makedirs(os.path.dirname(filePath), exist_ok=True)
        new_image.save(filePath)
        
        if os.path.exists(filePath):
            print(filename+" was saved successfully in the Maps folder.")


class StitchManager:
    def __init__(self):
        pass
    def init(self, lat, long):
        self.stitchCreator = StitchCreator(lat, long)
        self.lat, self.long = self.stitchCreator.FindCenter()
        self.status = False
        self.imgLength= self.stitchCreator.sz*3

        folderPath = os.path.dirname(os.path.realpath(__file__))

        for filename in os.listdir(os.path.join(folderPath, 'Maps')):
            if filename.startswith(str(self.lat) + "_" + str(self.long)) and filename.endswith('.png'):
                self.fname = str(folderPath)+'\\Maps\\'+filename
                info = os.path.basename(self.fname)[:-4].split('_')
                self.stitchLat = float(info[0])
                self.stitchLong = float(info[1])
                self.latFactorTop = float(info[2])
                self.latFactorMid = float(info[3])
                self.latFactorBott = float(info[4])
                self.img=cv2.imread(self.fname)
                self.status=True
        self.visionApp =VisionAppPC()

    def getStitchedImage(self):
        if self.status==True:
            self.visionApp.process(self.img)
            # self.contours=vApp.process(self.img)
            # self.img=cv2.drawContours(self.img, contours, -1, (0, 0, 255), 2)
            cv2.drawContours(self.img, self.visionApp.getContours(), -1, (0, 255, 0), 3)
            return self.img
        else:            
            blank_image = np.zeros((self.stitchCreator.sz*3,self.stitchCreator.sz*3,3), np.uint8)+255
            cv2.putText(
                img = blank_image,
                text = "No Map Found",
                org = (self.imgLength, self.imgLength),
                fontFace = cv2.FONT_HERSHEY_DUPLEX,
                fontScale = 3.0,
                color = (0,0,255),
                thickness = 3
                )
            return blank_image        
    def gps2Pixel(self,  gpsY, gpsX):
        patchSz = self.stitchCreator.sz
        longRng = self.stitchCreator.longfactor()
        minimumLong = self.long - longRng*1.5 
        pixelPerDegreeLong =patchSz/longRng
        px = pixelPerDegreeLong*(gpsX-minimumLong)
        px = int(max(min(patchSz*3, px), 0))

        midLattRange = [self.lat-self.latFactorMid/2,self.lat+self.latFactorMid/2 ]
        py = 0
        # if gpsY < midLattRange[0]:
        #     minimumLat_TopRow = self.lat - self.latFactorMid/2 - self.latFactorTop
        #     pixelPerDegreeLat = patchSz/self.latFactorTop
        #     py =  pixelPerDegreeLat*(gpsY  - minimumLat_TopRow) 
        # elif gpsY < midLattRange[1]:
        #     minimumLat_MidRow = self.lat - self.latFactorMid/2
        #     pixelPerDegreeLat = patchSz/self.latFactorMid
        #     py = patchSz   + pixelPerDegreeLat*(gpsY  - minimumLat_MidRow) 
        # else:
        #     minimumLat_BottRow = self.lat + self.latFactorMid/2
        #     pixelPerDegreeLat = patchSz/self.latFactorBott
        #     py =  patchSz*2+pixelPerDegreeLat*(gpsY  - minimumLat_BottRow)

        if gpsY > midLattRange[1]:
            maximumLat_TopRow = self.lat + self.latFactorMid/2 + self.latFactorTop
            pixelPerDegreeLat = patchSz/self.latFactorTop
            py =  pixelPerDegreeLat*(maximumLat_TopRow - gpsY) 
        elif gpsY > midLattRange[0]:
            maximumLat_MidRow = self.lat + self.latFactorMid/2
            pixelPerDegreeLat = patchSz/self.latFactorMid
            py = patchSz   + pixelPerDegreeLat*(maximumLat_MidRow - gpsY) 
        else:
            maxmimumLat_BottRow = self.lat - self.latFactorMid/2
            pixelPerDegreeLat = patchSz/self.latFactorBott
            py =  patchSz*2+pixelPerDegreeLat*(maxmimumLat_BottRow - gpsY )
        py = int(max(min(patchSz*3, py), 0))

        return px,py

    def pixel2Gps(self, px, py):
        if not self.status:
            return 0,0
        
        patchSz = self.stitchCreator.sz

        longRng = self.stitchCreator.longfactor()
        degreesPerPixelX =longRng/patchSz
        pointLng = 0
        if px < patchSz:
            pointLng = self.long - longRng + degreesPerPixelX*(px-patchSz/2)
        elif px < patchSz*2:
            px = px-patchSz
            pointLng = self.long + degreesPerPixelX*(px-patchSz/2)
        else:
            px = px-2*patchSz
            pointLng = self.long + longRng + degreesPerPixelX*(px-patchSz/2)

        pointLat = 0
        if py < patchSz:
            centerLat = self.lat + self.latFactorMid/2 + self.latFactorTop/2
            degreesPerPixelY = self.latFactorTop/640
            pointLat = centerLat - degreesPerPixelY*(py-patchSz/2)
        elif py < patchSz*2:
            py = py-patchSz
            degreesPerPixelY = self.latFactorMid/640
            pointLat = self.lat + degreesPerPixelY*(py-patchSz/2)
        else:
            py = py-2*patchSz
            centerLat = self.lat - self.latFactorMid/2 - self.latFactorBott/2
            degreesPerPixelY = self.latFactorBott/640
            pointLat = centerLat - degreesPerPixelY*(py-patchSz/2)
        return (pointLat, pointLng)

    def isPixelsInbound(self, px, py):
        if self.status==True:
            if self.visionApp.isPixelsInbound(px,py):
                return True 
            else:
                return False
        else:            
            return None 
    def getCenterPixel(self):
        return self.stitchCreator.sz*1.5,self.stitchCreator.sz*1.5 

if __name__ == '__main__':
    # sc = StitchCreator(43.263082,-79.918871)
    # sc.StitchAndSave()
    sm = StitchManager()
    sm.init(43.263082,-79.918871)

    img = sm.getStitchedImage()
    cv2.drawMarker(img, position=(960,960),color=(0,0,255),markerType=cv2.MARKER_CROSS,thickness=30)
    t = sm.pixel2Gps(60,1500)
    print(t)
    print(sm.gps2Pixel(t[0], t[1]))

    img = cv2.resize( img, (500,500))
    cv2.imshow('Stitched image',img)
    cv2.waitKey(0)

    print(sm.status)