import sys
sys.path.append("..")
import Utils
import sys, time, cv2
from multiprocessing import Process
import threading
from PyQt5.QtWidgets import QApplication, QWidget
import UserInterface #import the setup code for the app

def DisplayCameraView(platform):
    Camera = None
    if platform == 'PI':
        Camera = Utils.OperatorCameraPI()
    elif platform == 'SITL':
        Camera = Utils.OperatorCameraSITL()
    Camera.init()
    try:
        while True:
            Camera.read()
            cv2.imshow('DroneView', Camera.image)
            if cv2.waitKey(1)&0xFF == ord('q'):
                break
    except KeyboardInterrupt:
            Utils.LogDebug("Keyboard Interrupt")


if __name__ == '__main__':
    platform = 'SITL'
    droneInterface = None
    p_camera = None
    if platform == 'SITL':
        droneInterface = Utils.OperatorSocket(HOST="192.168.56.101")
        p_camera = Process(target=DisplayCameraView, args=('SITL',))
        p_camera.start()
    else:
        droneInterface = Utils.OperatorSocket(HOST="navio")
        p_camera = Process(target=DisplayCameraView, args=('PI',))
        p_camera.start()


    app = QApplication(sys.argv)
    uiApp = UserInterface.UIApp(droneInterface)

    #droneDisplayer = UserInterface.DroneDisplayer()
    #t1 = threading.Thread(target=uiApp.processDroneMessages, args=())
    #t1.start()

    app.exec_()

    # GUI Closed
    uiApp.close()
    p_camera.terminate()
