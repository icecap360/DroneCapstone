from multiprocessing import Process
import cv2
import queue, threading, time
import Utils.Common as Common

class Camera:
    def __init__(self):
        self.cap_receive, self.thread = None, None
        self.q = queue.Queue()
        self.image = None

    def read(self):
        self.image = self.q.get()
        
    def createCap(self):
        self.cap_receive = cv2.VideoCapture('udpsrc port=9000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
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
            if not self.q.empty():
                try:
                    self.q.get_nowait() # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            self.q.put(frame)

if __name__ == '__main__':
    DroneCamera = Camera()
    DroneCamera.createCap()
    try:
        while True:
            DroneCamera.read()
            cv2.imshow('DroneView', DroneCamera.image)
            if cv2.waitKey(1)&0xFF == ord('q'):
                break
    except KeyboardInterrupt:
            print("Keyboard Interrupt")
    cv2.destroyAllWindows()