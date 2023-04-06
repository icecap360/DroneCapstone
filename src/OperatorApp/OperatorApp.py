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

def StartDroneCameraDisplay1():  
    Camera = OperatorCameraPi()
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


if __name__ == '__main__':
    platform = 'PI'
    droneInterface = None
    p_camera = None
    if platform == 'SITL':
        droneInterface = Utils.MessageSocket("OPERATOR", HOST="192.168.56.101")
        # Camera = OperatorCameraSITL()
        # p_camera = Process(target=UserInterface.StartDroneCameraDisplay, args=(Camera, 'SITL',))
        # p_camera.start()
        Camera = OperatorCameraPi()
        # p_camera = Process(target=UserInterface.StartDroneCameraDisplay, args=(Camera,))
        p_camera = Process(target=StartDroneCameraDisplay1)
        p_camera.start()
    else:
        droneInterface = Utils.MessageSocket("OPERATOR",HOST="navio")
        Camera = OperatorCameraPi()
        #p_camera = Process(target=UserInterface.StartDroneCameraDisplay, args=(Camera,))
        p_camera = Process(target=StartDroneCameraDisplay1)
        time.sleep(0.5)
        p_camera.start()


    app = QApplication(sys.argv)

    mapWindow = MapWindow()

    uiApp = UserInterface.UIApp(droneInterface, mapWindow)

    #droneDisplayer = UserInterface.DroneDisplayer()
    #t1 = threading.Thread(target=uiApp.processDroneMessages, args=())
    #t1.start()

    app.exec_()

    # GUI Closed
    mapWindow.close(None)
    uiApp.close()
    p_camera.terminate()
