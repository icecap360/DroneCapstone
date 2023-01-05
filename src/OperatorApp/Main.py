from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QLineEdit, QGridLayout
import sys
import UserInterface #import the setup code for the app

class DroneApp(QtWidgets.QMainWindow, UserInterface.Ui_MainWindow): #the method for the main app
    def __init__(self, parent=None):
        super(DroneApp, self).__init__(parent)
        self.setupUi(self)

def main():
    app = QApplication(sys.argv)
    form = DroneApp()
    form.show()
    app.exec_()

if __name__ == '__main__':
    main()