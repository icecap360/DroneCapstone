from PyQt5 import QtCore, QtGui, QtWidgets, QtWebEngineWidgets, QtWebChannel#pip install  PyQtWebEngine
from PyQt5.QtCore import *
import json, os
from pathlib import Path
import io
import time
import logging
import folium #pip install folium
from folium.plugins import *
from folium.features import *
import PyQtInterface
from threading import Semaphore

class MapManager(QtCore.QObject):
    def __init__(self, window, initLatt, initLong):
        super().__init__(window)
        self.droneLatt, self.desLatt = initLatt, initLatt
        self.droneLong, self.desLong = initLong, initLong
        self.semDrone= Semaphore()
        self.semDes = Semaphore()
    def setDroneLattLong(self, lattidue, longitude):
        self.semDrone.acquire()
        self.droneLatt = lattidue
        self.droneLong = longitude
        self.semDrone.release()
    @QtCore.pyqtSlot(result=float)
    def getDroneLatt(self):
        self.semDrone.acquire()
        t= self.droneLatt
        self.semDrone.release()
        return t
    @QtCore.pyqtSlot(result=float)
    def getDroneLong(self):
        self.semDrone.acquire()
        t = self.droneLong
        self.semDrone.release()
        return t
    @QtCore.pyqtSlot(str)
    def processClick(self, json_data):
        data = json.loads(json_data)
        self.semDes.acquire()
        self.desLatt = data["lat"]
        self.desLong =  data["lng"]
        self.semDes.release()
    def getDesLatt(self):
        self.semDes.acquire()
        t = self.desLatt
        self.semDes.release()
        return t
    def getDesLong(self):
        self.semDes.acquire()
        t = self.desLong
        self.semDes.release()
        return t

class UIController(PyQtInterface.PyQtController):
    def setupUi(self, MainWindow):
        super().setupUi(MainWindow)

        self.mapManager = MapManager(self, 43.2617, -79.9228)
        
        ##folium start
        # coordinate = (43.2617, -79.9228)
        # self.m = folium.Map(
        # 	tiles='https://www.google.cn/maps/vt?lyrs=s@189&gl=cn&x={x}&y={y}&z={z}',
        #     attr='Google Satellite Map',
        #     zoom_start=16,
        #     max_zoom=22,
        #     location=coordinate
        # )
        # iconeDroneLoc = folium.Icon( icon='fa-circle', color="blue",prefix='fa')
        # folium.Marker(location=coordinate,popup='Drone Location', icon=iconeDroneLoc).add_to(self.m)
        # iconeDesLoc = folium.Icon( icon='fa-location-crosshairs', color="red",prefix='fa')
        # folium.Marker(location=coordinate,popup='Target Location', icon=iconeDesLoc).add_to(self.m)
        #fmtr = "function(num) {return L.Util.formatNum(num, 3) + ' º ';};"
        #MousePosition(position='topright', separator=' | ', prefix="Mouse:",
        #lat_formatter=fmtr, lng_formatter=fmtr).add_to(self.m)
        #ClickForMarker(popup='<b>Lat:</b> ${lat}<br /><b>Lon:</b> ${lng}').add_to(self.m)
        
        self.webView = QtWebEngineWidgets.QWebEngineView(self.centralwidget)

        channel = QtWebChannel.QWebChannel(self.webView)
        channel.registerObject("mapManager", self.mapManager)
        self.webView.page().setWebChannel(channel)

        #self.m.save('index.html')
        filename = os.fspath(Path(__file__).resolve().parent / "index.html")
        url = QtCore.QUrl.fromLocalFile(filename)
        self.webView.load(url)
        #self.webView.setHtml(data.getvalue().decode())
        self.gridLayout_1.addWidget(self.webView, 1, 1, 2, 1)

        self.gridLayout_1.setColumnStretch(1, 1)

class UIDisplayer(PyQtInterface.PyQtDisplayer):
    pass