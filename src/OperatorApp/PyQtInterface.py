# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Drone_App.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


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

class QTextEditLogger(logging.Handler): #update log file in logs textbox
    def __init__(self, parent):
        super().__init__()
        self.widget = QtWidgets.QPlainTextEdit(parent)
        self.widget.setReadOnly(True)
    def emit(self, record):
        msg = self.format(record)
        self.widget.appendPlainText(msg)
        self.widget.moveCursor(QtGui.QTextCursor.End)
class aThread(QThread): #enable background processing
    updt_chk = pyqtSignal(int)
    def run(self):
        x = 0
        while True:
            time.sleep(2)
            self.updt_chk.emit(x)
            x+=1
            if x > 3:
                x = 0

class PyQtController(object):
    def setupUi(self, Controller):
        Controller.setObjectName("Controller")
        Controller.setEnabled(True)
        Controller.resize(1000, 530)
        Controller.setMinimumSize(QtCore.QSize(1000, 0))
        self.centralwidget = QtWidgets.QWidget(Controller)
        self.centralwidget.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.centralwidget.setObjectName("centralwidget")
        self.grdlayController = QtWidgets.QGridLayout(self.centralwidget)
        self.grdlayController.setObjectName("grdlayController")
        self.horizontalLayout2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout2.setObjectName("horizontalLayout2")
        self.frmArm = QtWidgets.QFormLayout()
        self.frmArm.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.frmArm.setObjectName("frmArm")
        self.lblArmed = QtWidgets.QLabel(self.centralwidget)
        self.lblArmed.setObjectName("lblArmed")
        self.frmArm.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lblArmed)
        self.txtArmed = QtWidgets.QLabel(self.centralwidget)
        self.txtArmed.setObjectName("txtArmed")
        self.frmArm.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.txtArmed)
        self.horizontalLayout2.addLayout(self.frmArm)
        self.frmArdupilotMode = QtWidgets.QFormLayout()
        self.frmArdupilotMode.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.frmArdupilotMode.setHorizontalSpacing(6)
        self.frmArdupilotMode.setObjectName("frmArdupilotMode")
        self.lblArdupilotMode = QtWidgets.QLabel(self.centralwidget)
        self.lblArdupilotMode.setObjectName("lblArdupilotMode")
        self.frmArdupilotMode.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lblArdupilotMode)
        self.txtArdupilotMode = QtWidgets.QLabel(self.centralwidget)
        self.txtArdupilotMode.setObjectName("txtArdupilotMode")
        self.frmArdupilotMode.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.txtArdupilotMode)
        self.horizontalLayout2.addLayout(self.frmArdupilotMode)
        self.frmRelativeX = QtWidgets.QFormLayout()
        self.frmRelativeX.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.frmRelativeX.setObjectName("frmRelativeX")
        self.lblRelativeX = QtWidgets.QLabel(self.centralwidget)
        self.lblRelativeX.setObjectName("lblRelativeX")
        self.frmRelativeX.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lblRelativeX)
        self.txtRelativeX = QtWidgets.QLabel(self.centralwidget)
        self.txtRelativeX.setObjectName("txtRelativeX")
        self.frmRelativeX.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.txtRelativeX)
        self.horizontalLayout2.addLayout(self.frmRelativeX)
        self.frmRelativeY = QtWidgets.QFormLayout()
        self.frmRelativeY.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.frmRelativeY.setObjectName("frmRelativeY")
        self.lblRelativeY = QtWidgets.QLabel(self.centralwidget)
        self.lblRelativeY.setObjectName("lblRelativeY")
        self.frmRelativeY.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lblRelativeY)
        self.txtRelativeY = QtWidgets.QLabel(self.centralwidget)
        self.txtRelativeY.setObjectName("txtRelativeY")
        self.frmRelativeY.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.txtRelativeY)
        self.horizontalLayout2.addLayout(self.frmRelativeY)
        self.frmAltitude = QtWidgets.QFormLayout()
        self.frmAltitude.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.frmAltitude.setHorizontalSpacing(6)
        self.frmAltitude.setObjectName("frmAltitude")
        self.lblAltitude = QtWidgets.QLabel(self.centralwidget)
        self.lblAltitude.setObjectName("lblAltitude")
        self.frmAltitude.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lblAltitude)
        self.txtAltitude = QtWidgets.QLabel(self.centralwidget)
        self.txtAltitude.setObjectName("txtAltitude")
        self.frmAltitude.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.txtAltitude)
        self.horizontalLayout2.addLayout(self.frmAltitude)
        self.grdlayController.addLayout(self.horizontalLayout2, 2, 0, 1, 2)
        self.verticalLayout2 = QtWidgets.QVBoxLayout()
        self.verticalLayout2.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.verticalLayout2.setObjectName("verticalLayout2")
        self.lblLogs = QtWidgets.QLabel(self.centralwidget)
        self.lblLogs.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.lblLogs.setObjectName("lblLogs")
        self.verticalLayout2.addWidget(self.lblLogs)

        # self.txtLogs = QtWidgets.QTextEdit(self.centralwidget)
        # self.txtLogs.setReadOnly(True)
        # self.txtLogs.setObjectName("txtLogs")
        # self.verticalLayout2.addWidget(self.txtLogs)

        self.lblLogs = QtWidgets.QLabel(self.centralwidget)
        self.lblLogs.setObjectName("label_6")
        self.logTextBox = QTextEditLogger(self.centralwidget)
        self.textEdit = self.logTextBox.widget
        self.textEdit.setReadOnly(True)
        self.textEdit.setObjectName("textEdit")
        self.verticalLayout2.addWidget(self.lblLogs)
        self.verticalLayout2.addWidget(self.textEdit)

        self.grdlayController.addLayout(self.verticalLayout2, 4, 0, 1, 1)
        self.verticalLayout1 = QtWidgets.QVBoxLayout()
        self.verticalLayout1.setObjectName("verticalLayout1")
        self.btnConnect = QtWidgets.QPushButton(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnConnect.sizePolicy().hasHeightForWidth())
        self.btnConnect.setSizePolicy(sizePolicy)
        self.btnConnect.setObjectName("btnConnect")
        self.verticalLayout1.addWidget(self.btnConnect)
        self.frmHeight = QtWidgets.QFormLayout()
        self.frmHeight.setObjectName("frmHeight")
        self.lblMinHover = QtWidgets.QLabel(self.centralwidget)
        self.lblMinHover.setObjectName("lblMinHover")
        self.frmHeight.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lblMinHover)
        self.spboxMinHover = QtWidgets.QSpinBox(self.centralwidget)
        self.spboxMinHover.setObjectName("spboxMinHover")
        self.frmHeight.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.spboxMinHover)
        self.lblDesiredHover = QtWidgets.QLabel(self.centralwidget)
        self.lblDesiredHover.setObjectName("lblDesiredHover")
        self.frmHeight.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.lblDesiredHover)
        self.spboxDesiredHover = QtWidgets.QSpinBox(self.centralwidget)
        self.spboxDesiredHover.setObjectName("spboxDesiredHover")
        self.frmHeight.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.spboxDesiredHover)
        self.lblMaxHover = QtWidgets.QLabel(self.centralwidget)
        self.lblMaxHover.setObjectName("lblMaxHover")
        self.frmHeight.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.lblMaxHover)
        self.spboxMaxHover = QtWidgets.QSpinBox(self.centralwidget)
        self.spboxMaxHover.setObjectName("spboxMaxHover")
        self.frmHeight.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.spboxMaxHover)
        self.verticalLayout1.addLayout(self.frmHeight)
        self.btnConfiguration = QtWidgets.QPushButton(self.centralwidget)
        self.btnConfiguration.setObjectName("btnConfiguration")
        self.verticalLayout1.addWidget(self.btnConfiguration)
        self.hlayArm = QtWidgets.QHBoxLayout()
        self.hlayArm.setObjectName("hlayArm")
        self.btnArm = QtWidgets.QPushButton(self.centralwidget)
        self.btnArm.setCheckable(False)
        self.btnArm.setChecked(False)
        self.btnArm.setObjectName("btnArm")
        self.hlayArm.addWidget(self.btnArm)
        self.btnDisarm = QtWidgets.QPushButton(self.centralwidget)
        self.btnDisarm.setObjectName("btnDisarm")
        self.hlayArm.addWidget(self.btnDisarm)
        self.verticalLayout1.addLayout(self.hlayArm)
        self.btnTakeOff = QtWidgets.QPushButton(self.centralwidget)
        self.btnTakeOff.setObjectName("btnTakeOff")
        self.verticalLayout1.addWidget(self.btnTakeOff)
        self.btnAutoExplore = QtWidgets.QPushButton(self.centralwidget)
        self.btnAutoExplore.setObjectName("btnAutoExplore")
        self.verticalLayout1.addWidget(self.btnAutoExplore)
        self.btnAutoMove = QtWidgets.QPushButton(self.centralwidget)
        self.btnAutoMove.setObjectName("btnAutoMove")
        self.verticalLayout1.addWidget(self.btnAutoMove)
        self.btnCompulsiveMove = QtWidgets.QPushButton(self.centralwidget)
        self.btnCompulsiveMove.setObjectName("btnCompulsiveMove")
        self.verticalLayout1.addWidget(self.btnCompulsiveMove)
        self.btnLand = QtWidgets.QPushButton(self.centralwidget)
        self.btnLand.setObjectName("btnLand")
        self.verticalLayout1.addWidget(self.btnLand)
        self.grdlayController.addLayout(self.verticalLayout1, 3, 0, 1, 1)
        self.horizontalLayout1 = QtWidgets.QHBoxLayout()
        self.horizontalLayout1.setSpacing(10)
        self.horizontalLayout1.setObjectName("horizontalLayout1")
        self.lblLogo = QtWidgets.QLabel(self.centralwidget)
        self.lblLogo.setObjectName("lblLogo")
        self.horizontalLayout1.addWidget(self.lblLogo)
        self.lblConnection = QtWidgets.QLabel(self.centralwidget)
        self.lblConnection.setWordWrap(False)
        self.lblConnection.setObjectName("lblConnection")
        self.horizontalLayout1.addWidget(self.lblConnection)
        self.frmDroneState = QtWidgets.QFormLayout()
        self.frmDroneState.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.frmDroneState.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.frmDroneState.setContentsMargins(0, -1, -1, -1)
        self.frmDroneState.setHorizontalSpacing(6)
        self.frmDroneState.setObjectName("frmDroneState")
        self.lblDroneState = QtWidgets.QLabel(self.centralwidget)
        self.lblDroneState.setObjectName("lblDroneState")
        self.frmDroneState.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lblDroneState)
        self.txtDroneState = QtWidgets.QLabel(self.centralwidget)
        self.txtDroneState.setObjectName("txtDroneState")
        self.frmDroneState.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.txtDroneState)
        self.horizontalLayout1.addLayout(self.frmDroneState)
        self.frmDroneHealth = QtWidgets.QFormLayout()
        self.frmDroneHealth.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.frmDroneHealth.setHorizontalSpacing(6)
        self.frmDroneHealth.setObjectName("frmDroneHealth")
        self.lblDroneHealth = QtWidgets.QLabel(self.centralwidget)
        self.lblDroneHealth.setObjectName("lblDroneHealth")
        self.frmDroneHealth.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lblDroneHealth)
        self.txtDroneHealth = QtWidgets.QLabel(self.centralwidget)
        self.txtDroneHealth.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.txtDroneHealth.setObjectName("txtDroneHealth")
        self.frmDroneHealth.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.txtDroneHealth)
        self.horizontalLayout1.addLayout(self.frmDroneHealth)
        self.frmUserErrors = QtWidgets.QFormLayout()
        self.frmUserErrors.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.frmUserErrors.setHorizontalSpacing(6)
        self.frmUserErrors.setObjectName("frmUserErrors")
        self.lblUserErrors = QtWidgets.QLabel(self.centralwidget)
        self.lblUserErrors.setObjectName("lblUserErrors")
        self.frmUserErrors.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.lblUserErrors)
        self.txtUserErrors = QtWidgets.QLabel(self.centralwidget)
        self.txtUserErrors.setObjectName("txtUserErrors")
        self.frmUserErrors.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.txtUserErrors)
        self.horizontalLayout1.addLayout(self.frmUserErrors)
        self.pbarBattery = QtWidgets.QProgressBar(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pbarBattery.sizePolicy().hasHeightForWidth())
        self.pbarBattery.setSizePolicy(sizePolicy)
        self.pbarBattery.setProperty("value", 24)
        self.pbarBattery.setObjectName("pbarBattery")
        self.horizontalLayout1.addWidget(self.pbarBattery)
        self.grdlayController.addLayout(self.horizontalLayout1, 0, 0, 1, 2)
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setObjectName("widget")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.grdlayController.addWidget(self.widget, 3, 1, 3, 1)
        self.grdlayController.setColumnStretch(1, 1)
        Controller.setCentralWidget(self.centralwidget)

        self.retranslateUi(Controller)

    def retranslateUi(self, Controller):
        self._translate = QtCore.QCoreApplication.translate
        Controller.setWindowTitle(self._translate("Controller", "MainWindow"))
        self.lblArmed.setText(self._translate("Controller", "Armed:"))
        self.txtArmed.setText(self._translate("Controller", "-"))
        self.lblArdupilotMode.setText(self._translate("Controller", "Ardupilot Mode:"))
        self.txtArdupilotMode.setText(self._translate("Controller", "-"))
        self.lblRelativeX.setText(self._translate("Controller", "Relative X:"))
        self.txtRelativeX.setText(self._translate("Controller", "-"))
        self.lblRelativeY.setText(self._translate("Controller", "Relative Y:"))
        self.txtRelativeY.setText(self._translate("Controller", "-"))
        self.lblAltitude.setText(self._translate("Controller", "Altitude:"))
        self.txtAltitude.setText(self._translate("Controller", "-"))
        self.lblLogs.setText(self._translate("Controller", "Logs"))
        self.btnConnect.setText(self._translate("Controller", "Connect"))
        self.lblMinHover.setText(self._translate("Controller", "Min Hover Height:"))
        self.lblDesiredHover.setText(self._translate("Controller", "Desired Hover Height:"))
        self.lblMaxHover.setText(self._translate("Controller", "Max Hover Height:"))
        self.btnConfiguration.setText(self._translate("Controller", "Configuration"))
        self.btnArm.setText(self._translate("Controller", "Arm"))
        self.btnDisarm.setText(self._translate("Controller", "Disarm"))
        self.btnTakeOff.setText(self._translate("Controller", "Take-Off"))
        self.btnAutoExplore.setText(self._translate("Controller", "Autonomous Explore"))
        self.btnAutoMove.setText(self._translate("Controller", "Autonomous Move"))
        self.btnCompulsiveMove.setText(self._translate("Controller", "Compulsive Move"))
        self.btnLand.setText(self._translate("Controller", "Land"))
        self.lblLogo.setText(self._translate("Controller", "Logo"))
        self.lblConnection.setText(self._translate("Controller", "Connection:"))
        self.lblDroneState.setText(self._translate("Controller", "Drone State:"))
        self.txtDroneState.setText(self._translate("Controller", "-"))
        self.lblDroneHealth.setText(self._translate("Controller", "Drone Health:"))
        self.txtDroneHealth.setText(self._translate("Controller", "-"))
        self.lblUserErrors.setText(self._translate("Controller", "User Errors:"))
        self.txtUserErrors.setText(self._translate("Controller", "-"))

    def checkWifi(self, val):
        if val == 0:
            self.lblConnection.setPixmap(QtGui.QPixmap("icons/wifi0.png").scaled(32, 32))
        elif val == 1:
            self.lblConnection.setPixmap(QtGui.QPixmap("icons/wifi1.png").scaled(32, 32))
        elif val == 2:
            self.lblConnection.setPixmap(QtGui.QPixmap("icons/wifi2.png").scaled(32, 32))
        else:          
            self.lblConnection.setPixmap(QtGui.QPixmap("icons/wifi3.png").scaled(32, 32))

#Subapp for displaying Camera and Parking Lot Occupancy with slider in between
class PyQtDisplayer(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(1600,400)

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")

        self.splitter = QtWidgets.QSplitter(self.centralwidget)
        self.splitter.setChildrenCollapsible(False)
        self.splitter.setObjectName("splitter")
        MainWindow.setStyle(QtWidgets.QStyleFactory.create('Cleanlooks'))

        self.pushButton_1 = QtWidgets.QPushButton(self.splitter)
        self.pushButton_1.setObjectName("pushButton_1")
        self.pushButton_2 = QtWidgets.QPushButton(self.splitter)
        self.pushButton_2.setObjectName("pushButton_2")

        self.horizontalLayout.addWidget(self.splitter)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "CamWindow"))
        self.pushButton_1.setText(_translate("MainWindow", "PushButton"))
        self.pushButton_2.setText(_translate("MainWindow", "PushButton"))

