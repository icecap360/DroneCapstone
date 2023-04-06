# Author: Zaid
# Date: December 2023 
# Purpose: Contains code to create the 3 User Interface windows

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
from MapWindow import MapWindow

def StartDroneCameraDisplay(Camera):
    # thread function to create the camera window of the UI  
    Camera.init()
    try:
        while True:
            ret = Camera.read()
            if ret:
                cv2.imshow('DroneView', Camera.image)
            if cv2.waitKey(1)&0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        LogDebug("Keyboard Interrupt")

class UIController(PyQtInterface.PyQtController):
    # This class connects the map window, message socket and user buttons
    # It inherits form PyQtController, so that the GUI layout can be changed independetly 
    # without having to change this module
     
    def setupUi(self, MainWindow, droneInterface, mapWindow):
        super().setupUi(MainWindow)
        self.droneIsConnected = False
        self.droneComm = DroneCommManager(droneInterface)

        self.timer = QTimer()
        self.timer.timeout.connect(self.droneComm.process)
        self.timer.start(500)
        self.droneCommInitialized = False
        self.droneComm.initConnSig.connect(self.droneComm.initialize)
        self.droneComm.finished.connect(self.droneComm.deleteLater)
        self.droneComm.sendAsyncSig.connect(self.droneComm.sendMessageAsync)
        self.droneComm.stopSig.connect(self.droneComm.close)
        self.droneComm.heartbeat.connect(self.processHeartbeat)
        self.droneComm.errorLog.connect(self.processErrorLog)
        self.droneComm.connectionStatus.connect(self.updateIsConnected)

        self.checkWifi(0)

        self.mapWindow = mapWindow
        
        self.btnConnect.clicked.connect(self.btnCb_connection)
        self.btnConfiguration.clicked.connect(self.btnCb_configure)
        self.btnArm.clicked.connect(self.btnCbArm)
        self.btnDisarm.clicked.connect(self.btnCbDisarm)
        self.btnTakeOff.clicked.connect(self.btnCbTakeoff)
        self.btnCompulsiveMove.clicked.connect(self.btnCbCompulsiveBtn)
        self.btnLand.clicked.connect(self.btnCbLandBtn)
        
        # Create the log box 
        # self.logTextBox = QTextEditLogger(self.centralwidget)
        # self.textEdit = self.logTextBox.widget
        # self.logTextBox.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        logging.getLogger().addHandler(self.logTextBox)
        logging.getLogger().setLevel(logging.ERROR)

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
    def btnCbArm(self):
        if self.getIsConnected() and self.mapWindow.windowCreated:
            self.droneComm.sendAsyncSig.emit({'Type':'Arm'})
    def btnCbDisarm(self):
        if self.getIsConnected():
            self.droneComm.sendAsyncSig.emit({'Type':'Disarm'})
    def btnCbTakeoff(self):
        if self.getIsConnected() and self.mapWindow.windowCreated:
            self.droneComm.sendAsyncSig.emit({'Type':'Takeoff'})
    def btnCbAutoExplore(self):
        if self.getIsConnected():
            self.droneComm.sendAsyncSig.emit({'Type':'AutonomousExplore'})
    def moveCommand(self, command):
        # helper function to simplify sending a move command
        if 'yes' in command.text().lower():
            self.droneComm.sendAsyncSig.emit(
                    {'Type':'CompulsiveMove','Lat':self.desLoc[0],'Long':self.desLoc[1], 'w':float(0)}
                    ) 
    def btnCbCompulsiveBtn(self):
        if self.getIsConnected():
            self.desLoc = self.mapWindow.getDesLocGps()
            if self.desLoc == None:
                return
            locationValid = self.mapWindow.isDesLocInbound()
            if not locationValid:
                # prompt the user if they are sure they want to proceed
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
    def btnCbLandBtn(self):
        if self.getIsConnected():
            self.droneComm.sendAsyncSig.emit({'Type':'Land'})

    def setLabelText(self, obj, val):
        obj.setText(self._translate("Controller", str(val)))
    def close(self):
        self.mapWindow.close(None)
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
            if 'isOccupied' in data['OccMap'].keys() and 'Latitude' in data['OccMap'].keys() and 'Longitude' in data['OccMap'].keys():
                self.mapWindow.processOccupancy(data['OccMap']['isOccupied'],
                                                data['OccMap']['Latitude'], 
                                                data['OccMap']['Longitude'])
                print('Debug print - Prediction: ', end=' ')
                if data['OccMap']['isParkLot']:
                    print('ParkingLot', end=' ')
                if data['OccMap']['isNature']:
                    print('Nature', end=' ')
                if data['OccMap']['isOccupied']:
                    print('Occupied', end=' ')
                print('', end='\n')
        if 'RelAlt' in data.keys():
            self.setLabelText(self.txtAltitude, data['RelAlt'])
        if 'Arm' in data.keys():
            self.setLabelText(self.txtArmed, str(data['Arm']))
        if 'Lat' in data.keys() and 'Long' in data.keys():
            self.mapWindow.processDroneLoc(data['Lat'], data['Long'])
        if 'HomeLat' in data.keys() and 'HomeLong' in data.keys():
            #self.mapManager.setHomeLatLong(data['HomeLat'], data['HomeLong'])
            pass
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
        logging.error(str(data['Message']))

class DroneCommManager(QObject): 
    # Used for communicating with the drone, 
    # written using pyqt signal and slot events, so this module can run in the background
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


class UIApp(QtWidgets.QMainWindow, UIController): #the method for the main app
    # This is the main UI manager class
    def __init__(self, droneInterface, parent=None):
        mapWindow = MapWindow()

        super(UIApp, self).__init__(parent)
        self.setupUi(self, droneInterface, mapWindow)
        self.show()
 
