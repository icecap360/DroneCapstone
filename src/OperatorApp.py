import Utils
import sys, time, cv2
from multiprocessing import Process

def DisplayCameraView():
    DroneCamera = Utils.Camera()
    DroneCamera.createCap()

    try:
        while True:
            DroneCamera.read()
            cv2.imshow('DroneView', DroneCamera.image)
            if cv2.waitKey(1)&0xFF == ord('q'):
                break
    except KeyboardInterrupt:
            print("Keyboard Interrupt")

if __name__=='__main__':
    try:    
        #sys.path.append("..") # Adds higher directory to python modules path.
        #DisplayCameraView()
        #p_camera = Process(target=DisplayCameraView)
        #p_camera.start()
        #p_camera.join()

        ss = Utils.OperatorSocket()
        ss.init()
        while True:
            data = ss.getMessage()
            if data:
                print(data)
    except KeyboardInterrupt:
        ss.close()

