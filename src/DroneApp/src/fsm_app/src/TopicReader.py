import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String, Bool
from pygeodesy.geoids import GeoidPGM 
from threading import Semaphore

class TopicReader:
    '''
    This class is supposed to read from topics that come from the FCU and other nodes

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

        self.park_lot_det = Bool()
        self.sem_park_lot_det = Semaphore()

        self.des_loc_inbound = Bool()
        self.sem_des_loc_inbound = Semaphore()

        self.local_pose = PoseStamped()
        self.sem_local_pose = Semaphore()

        # Subscribe to topics at the end!
        self.state_sub = rospy.Subscriber("/mavros/state", State, callback = self.state_cb)
        self.global_pose_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, callback = self.global_pose_cb)
        self.diagnostic_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, callback = self.diagnostic_cb)
        self.park_lot_det_sub = rospy.Subscriber("/path_plan_app/parking_lot_detected", Bool, callback = self.park_lot_det_cb)
        self.des_loc_inbound_sub = rospy.Subscriber("/path_plan_app/desired_loc_inbound", Bool, callback = self.des_loc_inbound_cb)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = self.local_pose_cb)


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
    def park_lot_det_cb(self, msg):
        self.sem_park_lot_det.acquire()
        self.park_lot_det = msg
        self.sem_park_lot_det.release()
    def des_loc_inbound_cb(self, msg):
        self.sem_des_loc_inbound.acquire()
        self.des_loc_inbound = msg
        self.sem_des_loc_inbound.release()
    def local_pose_cb(self, msg):
        self.sem_local_pose.acquire()
        self.local_pose = msg
        self.sem_local_pose.release()
    
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
    def getParkLotDetected(self):
        self.sem_park_lot_det.acquire()
        t = self.park_lot_det.data
        self.sem_park_lot_det.release()
        return t
    def getDesLocInbound(self):
        self.sem_des_loc_inbound.acquire()
        t = self.des_loc_inbound.data
        self.sem_des_loc_inbound.release()
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
    def getLocalPose(self) -> PoseStamped:
        self.sem_local_pose.acquire()
        t = self.local_pose
        self.sem_local_pose.release()
        return t
    