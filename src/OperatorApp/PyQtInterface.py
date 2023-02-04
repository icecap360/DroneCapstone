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
        

class QTextEditLogger(logging.Handler): #update log file in logs textbox
    def __init__(self, parent):
        super().__init__()
        self.widget = QtWidgets.QPlainTextEdit(parent)
        self.widget.setReadOnly(True)

    def emit(self, record):
        msg = self.format(record)
        self.widget.appendPlainText(msg)
        self.widget.moveCursor(QtGui.QTextCursor.End)


class PyQtController(object): #setup for the main app; setup window frame settings, add widgets, set layouts
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setEnabled(True)

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.centralwidget.setObjectName("centralwidget")

        self.gridLayout_1 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_1.setObjectName("gridLayout_1")

        self.horizontalLayout_1 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_1.setObjectName("horizontalLayout_1")

        self.label_1 = QtWidgets.QLabel(self.centralwidget)
        self.label_1.setObjectName("label_1")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setObjectName("label_2")
        self.progressBar = QtWidgets.QProgressBar(self.centralwidget)
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")

        self.formLayout_1 = QtWidgets.QFormLayout()
        self.formLayout_1.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.formLayout_1.setObjectName("formLayout_1")
        self.formLayout_2 = QtWidgets.QFormLayout()
        self.formLayout_2.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.formLayout_2.setObjectName("formLayout_2")
        self.formLayout_3 = QtWidgets.QFormLayout()
        self.formLayout_3.setFormAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.formLayout_3.setObjectName("formLayout_3")

        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setObjectName("label_3")
        self.label_3t = QtWidgets.QLabel(self.centralwidget)
        self.label_3t.setObjectName("label_3t")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setObjectName("label_4")
        self.label_4t = QtWidgets.QLabel(self.centralwidget)
        self.label_4t.setObjectName("label_4t")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setObjectName("label_5")
        self.label_5t = QtWidgets.QLabel(self.centralwidget)
        self.label_5t.setObjectName("label_5t")

        self.formLayout_1.addRow(self.label_3,self.label_3t)
        self.formLayout_2.addRow(self.label_4,self.label_4t)
        self.formLayout_3.addRow(self.label_5,self.label_5t)

        self.horizontalLayout_1.addWidget(self.label_1)
        self.horizontalLayout_1.addWidget(self.label_2)
        self.horizontalLayout_1.addLayout(self.formLayout_1)
        self.horizontalLayout_1.addLayout(self.formLayout_2)
        self.horizontalLayout_1.addLayout(self.formLayout_3)
        self.horizontalLayout_1.addWidget(self.progressBar)
        self.gridLayout_1.addLayout(self.horizontalLayout_1, 0, 0, 1, 2)

        self.verticalLayout_1 = QtWidgets.QVBoxLayout()
        self.verticalLayout_1.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.verticalLayout_1.setObjectName("verticalLayout_1")

        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setObjectName("label_6")
        logTextBox = QTextEditLogger(self.centralwidget)
        self.textEdit = logTextBox.widget
        self.textEdit.setReadOnly(True)
        self.textEdit.setObjectName("textEdit")

        self.verticalLayout_1.addWidget(self.label_6)
        self.verticalLayout_1.addWidget(self.textEdit)
        self.gridLayout_1.addLayout(self.verticalLayout_1, 2, 0, 1, 1)

        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.formLayout_4 = QtWidgets.QFormLayout()
        self.formLayout_4.setObjectName("formLayout_4")

        self.pushButton_1 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_1.setObjectName("pushButton_1")
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setObjectName("label_7")
        self.label_8a = QtWidgets.QLabel(self.centralwidget)
        self.label_8a.setObjectName("label_8a")
        self.label_8b = QtWidgets.QLabel(self.centralwidget)
        self.label_8b.setObjectName("label_8b")
        self.label_8c = QtWidgets.QLabel(self.centralwidget)
        self.label_8c.setObjectName("label_8c")
        self.spinBox_1 = QtWidgets.QSpinBox(self.centralwidget)
        self.spinBox_1.setObjectName("spinBox_1")
        self.spinBox_2 = QtWidgets.QSpinBox(self.centralwidget)
        self.spinBox_2.setObjectName("spinBox_2")
        self.spinBox_3 = QtWidgets.QSpinBox(self.centralwidget)
        self.spinBox_3.setObjectName("spinBox_3")
        self.pushButton_2a = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2a.setObjectName("pushButton_2a")
        self.pushButton_2b = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2b.setObjectName("pushButton_2b")
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_5 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_6 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_6.setObjectName("pushButton_6")
        self.pushButton_7 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_7.setObjectName("pushButton_7")

        self.formLayout_4.addRow(self.label_8a,self.spinBox_1)
        self.formLayout_4.addRow(self.label_8b,self.spinBox_2)
        self.formLayout_4.addRow(self.label_8c,self.spinBox_3)

        self.verticalLayout_2.addWidget(self.pushButton_1)
        self.verticalLayout_2.addLayout(self.formLayout_4)
        self.verticalLayout_2.addWidget(self.pushButton_2a)
        self.verticalLayout_2.addWidget(self.pushButton_3)
        self.verticalLayout_2.addWidget(self.pushButton_2b)
        self.verticalLayout_2.addWidget(self.pushButton_4)
        self.verticalLayout_2.addWidget(self.pushButton_5)
        self.verticalLayout_2.addWidget(self.pushButton_6)
        self.verticalLayout_2.addWidget(self.pushButton_7)

        self.gridLayout_1.addLayout(self.verticalLayout_2, 1, 0, 1, 1)
        
        ##logging start
        logTextBox.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        logging.getLogger().addHandler(logTextBox)
        logging.getLogger().setLevel(logging.DEBUG)
        threads = aThread()
        threads.start()
        threads.finished.connect(self.close)
        threads.updt_chk.connect(self.msg)
        ##logging end

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        time.sleep(1)

    def msg(self, val):
        if val == 0:
            logging.debug('damn, a bug')
        elif val == 1:
            logging.info('something to remember')
        elif val == 2:
            logging.warning('that\'s not right')
        else:
            logging.error('foobar')

    def retranslateUi(self, MainWindow): #setup values for the given widgets and window
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "OperatorWindow"))
        self.label_1.setText(_translate("MainWindow", "Logo"))
        self.label_2.setText(_translate("MainWindow", "Connection:"))
        self.label_3.setText(_translate("MainWindow", "Drone State:"))
        self.label_3t.setText(_translate("MainWindow", "TextLabel"))
        self.label_4.setText(_translate("MainWindow", "Drone Health:"))
        self.label_4t.setText(_translate("MainWindow", "TextLabel"))
        self.label_5.setText(_translate("MainWindow", "Altitude:"))
        self.label_5t.setText(_translate("MainWindow", "TextLabel"))
        self.label_6.setText(_translate("MainWindow", "Logs"))
        self.label_7.setText(_translate("MainWindow", "Mode:"))
        self.label_8a.setText(_translate("MainWindow", "Min Hover Height:"))
        self.label_8b.setText(_translate("MainWindow", "Desired Hover Height:"))
        self.label_8c.setText(_translate("MainWindow", "Max Hover Height:"))
        self.pushButton_1.setText(_translate("MainWindow", "Connect"))
        self.pushButton_2a.setText(_translate("MainWindow", "Configuration"))
        self.pushButton_2b.setText(_translate("MainWindow", "Take-off"))
        self.pushButton_3.setText(_translate("MainWindow", "Arm"))
        self.pushButton_4.setText(_translate("MainWindow", "Autonomous Explore"))
        self.pushButton_5.setText(_translate("MainWindow", "Compulsive Move"))
        self.pushButton_6.setText(_translate("MainWindow", "Autonomous Move"))
        self.pushButton_7.setText(_translate("MainWindow", "Land"))

#Subapp for displaying Camera and Parking Lot Occupancy with slider in between
class PyQtDisplayer(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(800,400)

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

