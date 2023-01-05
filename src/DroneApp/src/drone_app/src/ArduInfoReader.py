import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray
from pygeodesy.geoids import GeoidPGM 
from threading import Semaphore

class ArduInfoReader:
    '''
    This class is supposed to read from topics that come from the FCU 

    https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level
    When controlling the FCU using global setpoints, you specify the altitude as 
    meters above mean sea level (AMSL). But when sensing the global position, the 
    altitude reported by ~global_position/global is specified as meters above the 
    WGS-84 ellipsoid. This can lead to differences in altitude that are dozens of 
    meters apart.
    '''
    
    def __init__(self):
        self._egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

        self.state = State()
        self.sem_state = Semaphore()

        self.global_pose = NavSatFix()
        self.sem_global_pose = Semaphore()

        self.diagnostics = DiagnosticArray()
        self.sem_diagnostic = Semaphore()

        # Subscribe to topics at the end!
        self.state_sub = rospy.Subscriber("/mavros/state", State, callback = self.state_cb)
        self.global_pose_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, callback = self.global_pose_cb)
        self.diagnostic_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, callback = self.diagnostic_cb)
        
    def convertToAMSL(self,lat, lon, height) -> float:
        return height - self._egm96.height(lat, lon)
    def convertToEllipsoid(self, lat, lon, height) -> float:
        return height + self._egm96.height(lat, lon)
    def state_cb(self,msg):
        self.sem_state.acquire()
        self.state = msg
        self.sem_state.release()
    def global_pose_cb(self, msg):
        self.sem_global_pose.acquire()
        msg.altitude = self.convertToAMSL(msg.latitude, msg.longitude, msg.altitude) 
        self.global_pose = msg
        self.sem_global_pose.release()
    def diagnostic_cb(self, msg):
        self.sem_diagnostic.acquire()
        self.diagnostics = msg
        self.sem_diagnostic.release()

    def getState(self) -> State:
        self.sem_state.acquire()
        t = self.state
        self.sem_state.release()
        return t
    def getPose(self) -> NavSatFix:
        self.sem_global_pose.acquire()
        t = self.global_pose
        self.sem_global_pose.release()
        return t
    def getDiagnostics(self):
        # True is normal
        self.sem_diagnostic.acquire()
        t = self.diagnostics.status
        self.sem_diagnostic.release()
        return t
    def getHealthStatus(self):
        # True is normal
        # In SITL, the z/altitude control and x/y position control seem to always be false, I dont know why, so I dont know if this is right
        self.sem_diagnostic.acquire()
        t = True
        for diag in self.diagnostics.status:
            if diag.level == 2:
                t = False
        self.sem_diagnostic.release()
        return t

    