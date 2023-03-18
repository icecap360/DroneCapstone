import sys
sys.path.append('..')
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
from threading import Semaphore, Event
from Utils.Common import * 
from geopy import distance
import haversine as hs
import threading
from math import sqrt, cos, radians
import cv2

def StartDroneCameraDisplay(Camera, platform="PI"):  
    Camera.init()
    try:
        while True:
            Camera.read()
            cv2.imshow('DroneView', Camera.image)
            if cv2.waitKey(1)&0xFF == ord('q'):
                break
    except KeyboardInterrupt:
            LogDebug("Keyboard Interrupt")

class MapManager(QtCore.QObject):
    def __init__(self, window, initLat, initLong):
        super().__init__(window)
        self.droneLat, self.desLat, self.homeLat = initLat, initLat, initLat
        self.droneLong, self.desLong, self.homeLong = initLong, initLong, initLong
        self.semDrone= Semaphore()
        self.semDes = Semaphore()
        self.semHome = Semaphore()
    def setDroneLatLong(self, latitdue, longitude):
        self.semDrone.acquire()
        self.droneLat = latitdue
        self.droneLong = longitude
        self.semDrone.release()
    def setHomeLatLong(self, latitdue, longitude):
        self.semHome.acquire()
        self.homeLat = latitdue
        self.homeLong = longitude
        self.semHome.release()
    @QtCore.pyqtSlot(result=float)
    def getDroneLat(self):
        self.semDrone.acquire()
        t= self.droneLat
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
        self.desLat = data["lat"]
        self.desLong =  data["lng"]
        self.semDes.release()
    def getDesLat(self):
        self.semDes.acquire()
        t = self.desLat
        self.semDes.release()
        return t
    def getDesLong(self):
        self.semDes.acquire()
        t = self.desLong
        self.semDes.release()
        return t
    def getRelXYDist(self):
        # Due to small distance, the simplified formula is used
        # Higher fidelity use libraries geopy or haversine
        self.semDes.acquire()
        self.semDrone.acquire()
        x = abs((radians(self.homeLong) - radians(self.desLong)) * cos(0.5 * (radians(self.droneLat) + radians(self.desLat))))
        y = abs(radians(self.homeLat) - radians(self.desLat))
        if self.homeLong > self.desLong:
            x = -x
        if self.homeLat > self.desLat:
            y = -y
        t = (6371*x*1000, 6371*y*1000)# 6371 is radius of the earth in km
        self.semDes.release()
        self.semDrone.release()
        return t


class UIController(PyQtInterface.PyQtController):
    def setupUi(self, MainWindow, droneInterface, displayWindow):
        super().setupUi(MainWindow)
        self.droneIsConnected = False
        #self.thread = QThread()
        self.droneComm = DroneCommManager(droneInterface)
        #self.droneComm.moveToThread(self.thread)

        self.timer = QTimer()
        self.timer.timeout.connect(self.droneComm.process)
        self.timer.start(500)
        #self.thread.started.connect(self.droneComm.run)
        self.droneCommInitialized = False
        self.droneComm.initConnSig.connect(self.droneComm.initialize)
        #self.droneComm.finished.connect(self.thread.quit)
        self.droneComm.finished.connect(self.droneComm.deleteLater)
        self.droneComm.sendAsyncSig.connect(self.droneComm.sendMessageAsync)
        self.droneComm.stopSig.connect(self.droneComm.close)
        self.droneComm.heartbeat.connect(self.processHeartbeat)
        self.droneComm.errorLog.connect(self.processErrorLog)
        self.droneComm.connectionStatus.connect(self.updateIsConnected)
        #self.thread.finished.connect(self.thread.deleteLater)
        #self.thread.start()

        self.checkWifi(0)
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
        #self.m.save('index.html')

        self.mapManager = MapManager(self, 43.2617, -79.9228)
        self.webView = QtWebEngineWidgets.QWebEngineView(self.centralwidget)
        channel = QtWebChannel.QWebChannel(self.webView)
        channel.registerObject("mapManager", self.mapManager)
        self.webView.page().setWebChannel(channel)
        filename = os.fspath(Path(__file__).resolve().parent / "index.html")
        url = QtCore.QUrl.fromLocalFile(filename)
        self.webView.load(url)
        #self.webView.setHtml(data.getvalue().decode())

        self.horizontalLayout_3.addWidget(self.webView)
        self.grdlayController.addWidget(self.widget, 3, 1, 3, 1)
        self.grdlayController.setColumnStretch(1, 1)
        #self.gridLayout_1.addWidget(self.webView, 1, 1, 2, 1)
        #self.gridLayout_1.setColumnStretch(1, 1)

        self.btnConnect.clicked.connect(self.btnCb_connection)
        self.btnConfiguration.clicked.connect(self.btnCb_configure)
        self.btnArm.clicked.connect(self.btnCb_arm)
        self.btnDisarm.clicked.connect(self.btnCb_disarm)
        self.btnTakeOff.clicked.connect(self.btnCb_takeoff)
        self.btnAutoExplore.clicked.connect(self.btnCb_autoExplore)
        self.btnAutoMove.clicked.connect(self.btnCb_autoMoveBtn)
        self.btnCompulsiveMove.clicked.connect(self.btnCb_compulsiveBtn)
        self.btnLand.clicked.connect(self.btnCb_landBtn)
        
        self.displayWindow = displayWindow

        ##logging start
        # self.logTextBox = QTextEditLogger(self.centralwidget)
        # self.textEdit = self.logTextBox.widget
        # self.logTextBox.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        logging.getLogger().addHandler(self.logTextBox)
        logging.getLogger().setLevel(logging.ERROR)
        # ##logging end
        QtCore.QMetaObject.connectSlotsByName(self)
    def btnCb_connection(self):
        if not self.droneCommInitialized:
            self.droneCommInitialized = True
            self.droneComm.initConnSig.emit()
    def btnCb_configure(self):
        if self.getIsConnected():
            self.droneComm.sendAsyncSig.emit({'Type':'Configure',
                'MinHoverHeight':self.spboxMinHover.value(),
                'DesiredHoverHeight':self.spboxDesiredHover.value(), 
                'MaxHoverHeight':self.spboxMaxHover.value()
                })
    def btnCb_arm(self):
        if self.getIsConnected():
            self.droneComm.sendAsyncSig.emit({'Type':'Arm'})
    def btnCb_disarm(self):
        if self.getIsConnected():
            self.droneComm.sendAsyncSig.emit({'Type':'Disarm'})
    def btnCb_takeoff(self):
        if self.getIsConnected():
            self.droneComm.sendAsyncSig.emit({'Type':'Takeoff'})
    def btnCb_autoExplore(self):
        if self.getIsConnected():
            self.droneComm.sendAsyncSig.emit({'Type':'AutonomousExplore'})
    def btnCb_autoMoveBtn(self):
        if self.getIsConnected():
            t = self.mapManager.getRelXYDist()
            self.droneComm.sendAsyncSig.emit({'Type':'AutonomousMove',
                'X':t[0],'Y':t[1], 'w':float(0)})
    def btnCb_compulsiveBtn(self):
        if self.getIsConnected():
            t = self.mapManager.getRelXYDist()
            self.droneComm.sendAsyncSig.emit({'Type':'CompulsiveMove',
                'X':t[0],'Y':t[1], 'w':float(0)})
    def btnCb_landBtn(self):
        if self.getIsConnected():
            self.droneComm.sendAsyncSig.emit({'Type':'Land'})

    def setLabelText(self, obj, val):
        obj.setText(self._translate("Controller", str(val)))
    def close(self):
        self.droneComm.stopSig.emit()
    def getIsConnected(self):
        return self.droneIsConnected
    def updateIsConnected(self, c):
        self.droneIsConnected = c
        if self.droneIsConnected:
            self.checkWifi(3)
        else:
            self.checkWifi(0)
    def processHeartbeat(self, data):
        self.getIsConnected()
        if 'State' in data.keys():
            self.setLabelText(self.txtDroneState, data['State'])
        if 'ArduMode' in data.keys():
            self.setLabelText(self.txtArdupilotMode, data['ArduMode'])
        if 'OccMap' in data.keys():
            self.displayWindow.updateOccupancyMap(data['OccMap'])
        if 'RelAlt' in data.keys():
            self.setLabelText(self.txtAltitude, data['RelAlt'])
        if 'Arm' in data.keys():
            self.setLabelText(self.txtArmed, str(data['Arm']))
        if 'Lat' in data.keys() and 'Long' in data.keys():
            self.mapManager.setDroneLatLong(data['Lat'], data['Long'])
        if 'HomeLat' in data.keys() and 'HomeLong' in data.keys():
            self.mapManager.setHomeLatLong(data['HomeLat'], data['HomeLong'])
        if 'RelX' in data.keys():
            self.setLabelText(self.txtRelativeX, round(data['RelX'], 4))
        if 'RelY' in data.keys():
            self.setLabelText(self.txtRelativeY, round(data['RelY'], 4))
        if 'BattPerc' in data.keys():
            self.pbarBattery.setProperty("value", data['BattPerc']*100)
        if 'UserErr' in data.keys():
            self.setLabelText(self.txtUserErrors, UserErrorCode(data['UserErr']).name)
        if 'HealthStatus' in data.keys():
            self.setLabelText(self.txtDroneHealth, HealthStatusCode(data['HealthStatus']).name)
    def processErrorLog(self, data):
        logging.error('Error: '+str(data['Message']))

class DroneCommManager(QObject): #enable background processing
    initConnSig = QtCore.pyqtSignal()
    sendAsyncSig = QtCore.pyqtSignal(dict)
    stopSig = QtCore.pyqtSignal()
    connectionStatus = QtCore.pyqtSignal(bool)
    finished = QtCore.pyqtSignal()
    heartbeat = QtCore.pyqtSignal(dict)
    errorLog = QtCore.pyqtSignal(dict)
    stopMsgProcess = Event()
    droneInterface = None

    def __init__(self, droneInterface):
        super(DroneCommManager, self).__init__()
        self.droneInterface = droneInterface
        self.status = False
        self.prevstatus = False

    @QtCore.pyqtSlot()
    def initialize(self):
        if not self.droneInterface.isInitialized():
            self.droneInterface.init()
    @QtCore.pyqtSlot()
    def close(self):
        self.stopMsgProcess.set()
    @QtCore.pyqtSlot(dict)
    def sendMessageAsync(self, msg:dict):
        self.droneInterface.sendMessageAsync(msg)
    @QtCore.pyqtSlot()
    def process(self):
        self.status = self.droneInterface.isConnected()
        if self.prevstatus != self.status:
            self.connectionStatus.emit(self.status)
        self.prevstatus = self.status
        if not self.status:
            return
        data = self.droneInterface.getMessage()
        if data:
            if data['Type'] == 'Heartbeat':
                self.heartbeat.emit(data)
            if data['Type'] == 'ErrorLog':
                self.errorLog.emit(data)
    def run(self):
        while True:
            if self.stopMsgProcess.is_set():
                break
            self.status = self.droneInterface.isConnected()
            if self.prevstatus != self.status:
                self.connectionStatus.emit(self.status)
            if not self.status:
                continue
            data = self.droneInterface.getMessage()
            if data:
                if data['Type'] == 'Heartbeat':
                    self.heartbeat.emit(data)
                if data['Type'] == 'ErrorLog':
                    self.errorLog.emit(data)
        self.finished.emit()

class UIDisplayer( PyQtInterface.PyQtDisplayer):
    def updateOccupancyMap(self, occMap):
        pass
        #LogDebug('UIDisplayer\n'+str(occMap))

class UIApp(QtWidgets.QMainWindow, UIController): #the method for the main app
    def __init__(self, droneInterface, parent=None):
        displayWindow = DroneDisplayer()
        super(UIApp, self).__init__(parent)
        self.setupUi(self, droneInterface, displayWindow)
        self.show()
        
class DroneDisplayer(QtWidgets.QMainWindow, UIDisplayer): #the method for the sub app
    def __init__(self, parent=None, child=None):
        super().__init__()
        self.setupUi(self)
        #self.show()
