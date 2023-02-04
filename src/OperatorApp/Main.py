from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget
import sys
import UserInterface #import the setup code for the app

class DroneApp(QtWidgets.QMainWindow, UserInterface.UIController): #the method for the main app
    def __init__(self, parent=None):
        super(DroneApp, self).__init__(parent)
        self.setupUi(self)
        self.camocc = CamOccApp()
        self.camocc.show()
        
class CamOccApp(QtWidgets.QMainWindow, UserInterface.UIDisplayer): #the method for the sub app
    def __init__(self, parent=None, child=None):
        super().__init__()
        self.setupUi(self)

def main():
    app = QApplication(sys.argv)
    drone = DroneApp()
    drone.show()
    app.exec_()

if __name__ == '__main__':
    main()