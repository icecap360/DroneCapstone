import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import time
from Stitch import StitchManager

class StaticMarkerGenerator:
    def __init__(self, symbol, size, color):
        self.symbol = symbol
        self.size = size
        self.color = color
    def add(self, x, y):
            plt.plot(x, y, marker=self.symbol,
                            markersize=self.size,
                            markerfacecolor=self.color,
                            markeredgecolor='none')
    def addTuple(self, tuple):
        self.add(tuple[0], tuple[1])

class MoveableMarker:
    def __init__(self, symbol, size, color):
        self.symbol = symbol
        self.size = size
        self.color = color
        self.x,self.y = -1,-1
        self.marker = None
    def move(self, x, y):
        if self.marker:
            self.marker.pop(0).remove()
        self.x,self.y = x,y
        self.marker = plt.plot(x, y, marker=self.symbol,
                            markersize=self.size,
                            markerfacecolor=self.color,
                            markeredgecolor=self.color, markeredgewidth=.1)
    def moveEvent(self, event):
        self.move(event.xdata, event.ydata)
    def moveTuple(self, tuple):
        self.move(tuple[0], tuple[1])

class MarkerList:
    def __init__(self, symbol, size, color, length=10):
        self.symbol = symbol
        self.size = size
        self.color = color
        self.length = length
        self.markers = [None for _ in range(length)]
        self.prevX, self.prevY = -1,-1
    def update(self, x, y):
        if self.markers[0]:
            self.markers[0].pop(0).remove()
        self.markers.pop(0)
        if (self.prevX != -1 and self.prevY != -1):
            self.markers.append(plt.plot(self.prevX, self.prevY, marker=self.symbol,
                            markersize=self.size,
                            markerfacecolor=self.color,
                            markeredgecolor=self.color, markeredgewidth=.1))
        else:
            self.markers.append(0)
        self.prevX, self.prevY = x,y

    def update(self, moveableMarker):
        if self.markers[0]:
            self.markers[0].pop(0).remove()
        self.markers.pop(0)
        self.markers.append(plt.plot(moveableMarker.x, moveableMarker.y, marker=self.symbol,
                            markersize=self.size,
                            markerfacecolor=self.color,
                            markeredgecolor=self.color, markeredgewidth=.1))

class MapWindow:
    def __init__(self):
        self.windowCreated = False
        self.gpsX, self.gpsY = -1,-1
    def init(self,  gpsX, gpsY):   
        if self.windowCreated:
            return     
        
        self.fig = plt.figure(figsize=(7,7))#figsize=(len(img) * pixel_per_bar / dpi, 2), dpi=dpi)
        self.ax = self.fig.add_axes([0, 0, 1, 1])  # span the whole figure
        self.fig.canvas.set_window_title('Satellite Map')
        self.ax.set_axis_off()

        self.stitchManager = StitchManager()
        self.stitchManager.init(gpsX, gpsY)

        if self.stitchManager.status:
            self.gpsX, self.gpsY = gpsX, gpsY
        elif self.gpsX != -1 and self.gpsY != -1:
            # if the current drone location does not have a sitch,
            # use the previous drone successful stitch
            self.stitchManager.init(gpsX, gpsY)
        else:
            # if there is no previous stich, return
            self.ax.imshow(self.stitchManager.getStitchedImage())
            self.windowCreated = False
            return
        
        # setup figure size

        self.unoccupiedGen = StaticMarkerGenerator("o", 7, (102/255, 255/255, 102/255, 1))
        self.occupiedGen = StaticMarkerGenerator("o", 7, (255/255, 0/255, 0/255, 1))
        self.droneLocMarker = MoveableMarker("d", 14, (0/255, 0/255, 255/255, 1))
        self.desLocMarker = MoveableMarker("X", 14, (255/255, 195/255, 0/255, 1))
        self.desLocMarker.moveTuple(self.stitchManager.getCenterPixel())
        self.trace = MarkerList("o", 7,(0/255, 255/255, 255/255, 1))
        self.fig.canvas.mpl_connect('button_press_event', self.desLocMarker.moveEvent)
        self.fig.canvas.mpl_connect('close_event', self.on_close)

        self.ax.imshow(self.stitchManager.getStitchedImage())
        self.windowCreated = True
        plt.ion()
        plt.show()

    def on_close(self, event):
        self.windowCreated = False
    def isDesLocInbound(self):
        if not self.windowCreated:
            return False
        return self.stitchManager.LocationInbound(self.desLocMarker.x, self.desLocMarker.y)
    def getDesLocGps(self):
        if not self.windowCreated:
            return None
        return self.stitchManager.pixel2Gps(self.desLocMarker.x, self.desLocMarker.y)
    def processDroneLoc(self, droneGpsLat, droneGpsLong):
        if not self.windowCreated:
            # recreate the stitch window if it was closed
            self.init(droneGpsLat, droneGpsLong)
        self.droneLocMarker.moveTuple(self.stitchManager.gps2Pixel(droneGpsLat, droneGpsLong))
        self.trace.update(self.droneLocMarker)
    
    def processOccupancy(self,  isOccupied=None, occupancyLat=None, occupancyLong=None):
        if not self.windowCreated:
            return
        if isOccupied==False:
            self.unoccupiedGen.addTuple(self.stitchManager.gps2Pixel(occupancyLat,occupancyLong))
        elif isOccupied == True:
            self.occupiedGen.addTuple(self.stitchManager.gps2Pixel(occupancyLat,occupancyLong))
    
    def end(self):
        plt.close()


if __name__=='__main__':
    mapWindow = MapWindow()
    for i in range(20):
        mapWindow.processDroneLoc(43.26, -79.902)
        mapWindow.processOccupancy(False, 43.26, -79.901)
        mapWindow.processOccupancy(True, 43.261, -79.902)
        print(mapWindow.getDesLocGps())
        x=input()
        