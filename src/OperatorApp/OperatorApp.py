import sys
sys.path.append('..')
import Utils
import sys, time, cv2
from multiprocessing import Process

def DisplayCameraView():
    DroneCamera = Utils.Camera()
    DroneCamera.init()

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

        ss = Utils.OperatorSocket(HOST="192.168.56.101")
        ss.init()
        ss.sendMessageSync({'Type':'Launch','Mode':'Normal'})
        x = input()
        while x != '-1':
            print('processing ', x)
            #ss.sendMessageSync({'Type':'AutonomousMove','X':float(x),'Y':float(x), 'w':float(0)})
            ss.sendMessageSync({'Type':'AutonomousExplore'})
            x = input()
        ss.sendMessageSync({'Type':'Land'})
        print('Message sent')
        while True:
            data = ss.getMessage()
            if data:
                print(data)
    except KeyboardInterrupt:
        ss.close()