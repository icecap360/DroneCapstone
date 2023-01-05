from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
import sys
import Drone_Controls1 #setup code for the app

class DroneApp(QtWidgets.QMainWindow, Drone_Controls1.Ui_MainWindow): #the method for the main app
    def __init__(self, parent=None):
        super(DroneApp, self).__init__(parent)
        self.setupUi(self)
        
class Loginpage(QWidget): #the login page method for the app
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Login')
        self.resize(350,200)
        self.user_obj = QLineEdit()
        self.user_pwd = QLineEdit()
        self.user_pwd.setEchoMode(QLineEdit.Password) #function to hide the text in the textfield
        button_login = QPushButton('Login')
        layout = QFormLayout() #the following layout adds a label, text field and button vertically
        layout.addRow(QLabel('<font size="8"> UserId: </font>'), self.user_obj)
        layout.addRow(QLabel('<font size="8"> Password: </font>'), self.user_pwd)
        layout.addRow(button_login)
        self.setLayout(layout)
        button_login.clicked.connect(self.clicks)
        
    def clicks(self, checked): #the following method checks if user id and password is correcct
        #if self.user_obj.text() == "admin" and self.user_pwd.text() == "admin":
            self.w = DroneApp()
            self.w.show()
            self.close()
        # else:
            # QtWidgets.QMessageBox.warning(self, 'Error', 'Bad user or password')

def main(): #setup app and form to begin
    app = QApplication(sys.argv)
    form = Loginpage()
    form.show()
    app.exec_()

if __name__ == '__main__':
    main()