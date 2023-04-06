# Author: Ali
# Date: December 2023 
# Purpose: Contains code to send and receive camera images for both the drone and Operators PC


from multiprocessing import Process
import cv2, io
from multiprocessing import Queue, Lock
import threading, time
import numpy as np
import Utils.Common as Common

class Camera:
    def __init__(self):
        self.lock = Lock()
        self.image = None
        self.rawImage = None
        self.newRawImage = False
        self.stopEvent = threading.Event()

    def read(self):
        ret = False
        self.lock.acquire()
        if self.newRawImage:
            self.image = self.rawImage
            self.newRawImage = False
            ret = True
        self.lock.release()
        return ret

    def init(self, cap_receive):
        self.cap_receive = cap_receive
        if not self.cap_receive.isOpened():
            Common.LogError('VideoCapture not opened')
            raise Exception('Video Capture failed to opened')
        self.thread = threading.Thread(target=self._backendReader)
        self.thread.daemon = True
        self.thread.start()

    def _backendReader(self):
        # thread in background to read images and store them in rawImage
        # reads frames as soon as they are available, keeping only most recent one
        while True:
            ret, frame = self.cap_receive.read()
            if not ret:
                Common.LogError('ret is not Okay')
                continue
            if self.stopEvent.is_set():
                break
            self.lock.acquire()
            self.rawImage = frame
            self.newRawImage =True
            self.lock.release()
    def close(self):
        self.stopEvent.set()

class OperatorCameraPi(Camera):
    def init(self):
        print('OperatorCameraPi')
        cap = cv2.VideoCapture('udpsrc port=9000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
        super().init(cap)
class OperatorCameraSITL(Camera):
    def init(self):
        cap = cv2.VideoCapture('udpsrc port=5600 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
        super().init(cap) 

class DroneCamera(Camera):
    def init(self):
        from picamera.array import PiRGBArray
        from picamera import PiCamera
        time.sleep(0.1)
        gst_pipe = "appsrc ! videoconvert ! v4l2h264enc ! video/x-h264,width=640,height=480,framerate=15/1,speed-preset=ultrafast ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.137.1 port=9000"
        self.cap_send = cv2.VideoWriter(gst_pipe, cv2.CAP_GSTREAMER, 0, 15, (640, 480), True)
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 10
        #self.camera.exposure_mode = 'sports'
        #self.camera.image_effect = 'colorbalance'
        #self.camera.image_effect_params = (1, 2, 1, 1)
        time.sleep(2.1)
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        self.rawCapture.truncate(0)
        self.readThread = threading.Thread(target=self._backendReader)
        self.readThread.daemon = True
        self.readThread.start()
        time.sleep(0.1)
    def send(self, img):
        self.cap_send.write(img)

    def _backendReader(self):
        # thread in background to read images and store them in rawImage
        # reads frames as soon as they are available, keeping only most recent one
        try:
            for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                if self.stopEvent.is_set():
                    break
                self.lock.acquire()
                self.rawImage = frame.array
                self.send(self.rawImage)
                self.newRawImage = True
                self.rawCapture.truncate(0)
                self.lock.release()

            self.cap_send.release()
        except KeyboardInterrupt:
            self.cap_send.release()



if __name__ == '__main__':
    platform = 'PI'
    if platform == 'PI':
        droneCamera = DroneCamera()
        droneCamera.init()
        try:
            while True: 
                time.sleep(1)
                #print('start time', time.time())
                #ret = droneCamera.read()
                #while not ret:
                #    ret = droneCamera.read()
                #print('end time', time.time())
                #cv2.imwrite('captured_image%s.png' %i, droneCamera.image)
                #droneCamera.send(droneCamera.image)
                #time.sleep(1)
        except KeyboardInterrupt:
            droneCamera.close()
            print('Keyboard Interrrupt')
    elif platform == 'OperatorPi':
        camera = OperatorCameraPi()
        camera.init()
        try:
            while True:
                camera.read()
                cv2.imshow('DroneView', camera.image)
                if cv2.waitKey(1)&0xFF == ord('q'):
                    break
        except KeyboardInterrupt:
                print("Keyboard Interrupt")
        cv2.destroyAllWindows()
