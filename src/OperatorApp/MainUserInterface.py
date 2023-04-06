# Author: Zaid
# Date:December 2022
# Purpose: Main User Interface module

import sys
sys.path.append("..")
import Utils
import sys, time
from multiprocessing import Process
import threading
from PyQt5.QtWidgets import QApplication, QWidget
import UserInterface #import the setup code for the app
from Utils import OperatorCameraPi, OperatorCameraSITL
from MapWindow import MapWindow
from Stitch import StitchManager
import cv2 


if __name__ == '__main__':
    platform = 'PI'
    droneInterface = None
    p_camera = None
    if platform == 'SITL':
        droneInterface = Utils.MessageSocket("OPERATOR", HOST="192.168.56.101")
        Camera = OperatorCameraSITL()
        p_camera = Process(target=UserInterface.StartDroneCameraDisplay, args=(Camera))
        p_camera.start()
        p_camera = Process(target=UserInterface.StartDroneCameraDisplay, args=(Camera))
    else:
        droneInterface = Utils.MessageSocket("OPERATOR",HOST="navio")
        Camera = OperatorCameraPi()
        p_camera = Process(target=UserInterface.StartDroneCameraDisplay, args=(Camera,))
        time.sleep(0.5)
        p_camera.start()


    app = QApplication(sys.argv)

    uiApp = UserInterface.UIApp(droneInterface)

    app.exec_()

    # GUI Closed
    uiApp.close()
    p_camera.terminate()
