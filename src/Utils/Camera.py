from multiprocessing import Process
import cv2
from multiprocessing import Queue, Lock
import threading, time
#import Utils.Common as Common

class Camera:
    def __init__(self):
        self.q = Queue()
        self.lock = Lock()
        self.image = None

    def read(self):
        ret = False
        self.lock.acquire()
        if not self.q.empty():
            ret = True
            self.image = self.q.get_nowait()
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
        # reads frames as soon as they are available, keeping only most recent one
        while True:
            ret, frame = self.cap_receive.read()
            if not ret:
                Common.LogError('ret is not Okay')
                continue
            try:
                self.lock.acquire()
                self.q.get_nowait() # discard previous (unprocessed) frame
                self.lock.release()
            except: # if QueueEmpty
                pass
            self.lock.acquire()
            self.q.put(frame)
            self.send(frame)
            self.lock.release()


class OperatorCameraPi(Camera):
    def init(self):
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
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 30
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        gst_pipe = "appsrc ! videoconvert ! v4l2h264enc ! video/x-h264,level=(string)4 ! ' \
          'h264parse ! matroskamux ! tcpserversink host=0.0.0.0 port=7000"
        gst_pipe = "appsrc ! ' \
          'h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.137.1 port=9000"
        #gst_pipe = "appsrc ! videoconvert ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.137.1 port=9000"
        #gst_pipe = "appsrc ! videoconvert ! v4l2h264enc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.137.1 port=9000"
        gst_pipe = "appsrc ! videoconvert ! v4l2h264enc ! video/x-h264,width=640,height=480,framerate=30/1 ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.137.1 port=9000"
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        self.cap_send = cv2.VideoWriter(gst_pipe, cv2.CAP_GSTREAMER, 0, 30, self.camera.resolution, True)
        self.thread = threading.Thread(target=self._backendReader)
        self.thread.daemon = True
        self.thread.start()
        time.sleep(0.1)
    def send(self, img):
        self.cap_send.write(img)

    def _backendReader(self):
        try:
            for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                self.lock.acquire()
                try:
                    self.q.get_nowait() # discard previous (unprocessed) frame
                except: # if QueueEmpty
                    pass
                self.q.put(frame.array)
                self.send(frame.array)
                self.lock.release()
                self.rawCapture.truncate(0) 
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
