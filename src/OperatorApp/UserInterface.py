import sys
sys.path.append('..')
from PyQt5 import QtCore, QtGui, QtWidgets, QtWebEngineWidgets, QtWebChannel#pip install  PyQtWebEngine
from PyQt5.QtWidgets import QMessageBox
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
import subprocess

def StartDroneCameraDisplay(Camera, platform="PI"):  
    # list_files = subprocess.Popen (
    #     ["gst-launch-1.0.exe","-v", "udpsrc port=9000 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=f"
    #      ])
    Camera.init()
    try:
        while True:
            ret = Camera.read()
            if ret:
                cv2.imshow('DroneView', Camera.image)
            print('cv2.waitKey(1)&0xFF ')
            if cv2.waitKey(1)&0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        LogDebug("Keyboard Interrupt")

class UIController(PyQtInterface.PyQtController):
    def setupUi(self, MainWindow, droneInterface, mapWindow):
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

        self.mapWindow = mapWindow
        
        self.btnConnect.clicked.connect(self.btnCb_connection)
        self.btnConfiguration.clicked.connect(self.btnCb_configure)
        self.btnArm.clicked.connect(self.btnCb_arm)
        self.btnDisarm.clicked.connect(self.btnCb_disarm)
        self.btnTakeOff.clicked.connect(self.btnCb_takeoff)
        self.btnCompulsiveMove.clicked.connect(self.btnCb_compulsiveBtn)
        self.btnLand.clicked.connect(self.btnCb_landBtn)
        
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
        if self.getIsConnected() and self.mapWindow.windowCreated:
            self.droneComm.sendAsyncSig.emit({'Type':'Arm'})
    def btnCb_disarm(self):
        if self.getIsConnected():
            self.droneComm.sendAsyncSig.emit({'Type':'Disarm'})
    def btnCb_takeoff(self):
        if self.getIsConnected() and self.mapWindow.windowCreated:
            self.droneComm.sendAsyncSig.emit({'Type':'Takeoff'})
    def btnCb_autoExplore(self):
        if self.getIsConnected():
            self.droneComm.sendAsyncSig.emit({'Type':'AutonomousExplore'})
    def moveCommand(self, command):
        if 'yes' in command.text().lower():
            self.droneComm.sendAsyncSig.emit(
                    {'Type':'CompulsiveMove','Lat':self.desLoc[0],'Long':self.desLoc[1], 'w':float(0)}
                    ) 
    def btnCb_compulsiveBtn(self):
        if self.getIsConnected():
            self.desLoc = self.mapWindow.getDesLocGps()
            if self.desLoc == None:
                return
            locationValid = self.mapWindow.isDesLocInbound()
            if not locationValid:
                msg = QMessageBox()
                msg.setWindowTitle("Compulsive Move")
                msg.setText("The internal satellite map suggests that the requested location is not within a parking lot."
                            " \nAre you sure you want to move to the location?")
                msg.setIcon(QMessageBox.Question)
                msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No) 
                msg.setDefaultButton(QMessageBox.Yes)
                msg.buttonClicked.connect(self.moveCommand)
                msg.exec_()
            else:
                self.droneComm.sendAsyncSig.emit(
                    {'Type':'CompulsiveMove','Lat':self.desLoc[0],'Long':self.desLoc[1], 'w':float(0)}
                    ) 
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
            pass
            #self.displayWindow.updateOccupancyMap(data['OccMap'])
        if 'RelAlt' in data.keys():
            self.setLabelText(self.txtAltitude, data['RelAlt'])
        if 'Arm' in data.keys():
            self.setLabelText(self.txtArmed, str(data['Arm']))
        if 'Lat' in data.keys() and 'Long' in data.keys():
            self.mapWindow.processDroneLoc(data['Lat'], data['Long'])
        if 'HomeLat' in data.keys() and 'HomeLong' in data.keys():
            pass
            #self.mapManager.setHomeLatLong(data['HomeLat'], data['HomeLong'])
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
    def __init__(self, droneInterface, mapWindow, parent=None):
        super(UIApp, self).__init__(parent)
        self.setupUi(self, droneInterface, mapWindow)
        self.show()
        
class DroneDisplayer(QtWidgets.QMainWindow, UIDisplayer): #the method for the sub app
    def __init__(self, parent=None, child=None):
        super().__init__()
        self.setupUi(self)
        #self.show()
