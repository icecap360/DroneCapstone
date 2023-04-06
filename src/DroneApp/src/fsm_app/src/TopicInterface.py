# Author: Zaid
# Date:November 2022
# Purpose: Module with all the topics that any operation states could need.

import rospy
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State, HomePosition, OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from sensor_msgs.msg import NavSatFix, BatteryState
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String, Bool, Float64
from pygeodesy.geoids import GeoidPGM 
from threading import Semaphore
from algo_app.msg import OccupancyMap

class TopicInterface:
    '''
    This class is supposed to read from topics that come from the FCU and other nodes
    All operation states should store its topic related behaviour here.

    https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level
    When controlling the FCU using global setpoints, you specify the altitude as 
    meters above mean sea level (AMSL). But when sensing the global position, the 
    altitude reported by ~global_position/global is specified as meters above the 
    WGS-84 ellipsoid. This can lead to differences in altitude that are dozens of 
    meters apart.
    '''
    
    def __init__(self):
        self.currStatePub = rospy.Publisher("/fsm_app/drone_state", String, queue_size=10)
        self.desPosePub = rospy.Publisher("/fsm_desired_pose/desired_pose", GeoPoseStamped, queue_size=10)
        
        self._egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

        self.state = State()
        self.semState = Semaphore()

        self.globalPose = NavSatFix()
        self.semGlobalPose = Semaphore()

        self.diagnostics = DiagnosticArray()
        self.semDiagnostic = Semaphore()

        self.parkLotDet = Bool()
        self.semParkLotDet = Semaphore()

        self.desLocInbound = Bool()
        self.semDesLocInbound = Semaphore()

        self.localPose = PoseStamped()
        self.semLocalPose = Semaphore()

        self.battery = BatteryState()
        self.semBattery = Semaphore()

        self.occupancyMap = OccupancyMap()
        self.semOccupancyMap = Semaphore()

        self.relAlt = Float64()
        self.semRelAlt = Semaphore()

        self.home_position = HomePosition()
        self.semHomePosition = Semaphore()

        # Subscribe to topics at the end!
        self.stateSub = rospy.Subscriber("/mavros/state", State, callback = self.stateCallbck)
        self.globalPoseSub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, callback = self.globalPoseCallbck)
        self.diagnosticSub = rospy.Subscriber("/diagnostics", DiagnosticArray, callback = self.diagnosticCallbck)
        self.parkLotDetSub = rospy.Subscriber("/algo_app/parking_lot_detected", Bool, callback = self.parkLotdetCallbck)
        self.desLocInboundSub = rospy.Subscriber("/algo_app/desired_loc_inbound", Bool, callback = self.desLocInboundCallbck)
        self.occupancyMapSub = rospy.Subscriber("/algo_app/occupancy_map", OccupancyMap, callback = self.occupancyMapCallbck)
        self.localPoseSub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = self.localPoseCallbck)
        self.homePositionSub = rospy.Subscriber("/mavros/home_position/home", HomePosition, callback = self.homePositionCallbck)
        self.batterySub = rospy.Subscriber("/mavros/battery", BatteryState, callback = self.batteryCallbck)
        self.relAltSub = rospy.Subscriber("/mavros/global_position/rel_alt", Float64, callback = self.relAltCallbck)

    def convertToAMSL(self,lat, lon, height) -> float:
        return height - self._egm96.height(lat, lon)
    def convertToEllipsoid(self, lat, lon, height) -> float:
        return height + self._egm96.height(lat, lon)
    def stateCallbck(self,msg):
        self.semState.acquire()
        self.state = msg
        self.semState.release()
    def globalPoseCallbck(self, msg):
        self.semGlobalPose.acquire()
        msg.altitude = self.convertToAMSL(msg.latitude, msg.longitude, msg.altitude) 
        self.globalPose = msg
        self.semGlobalPose.release()
    def diagnosticCallbck(self, msg):
        self.semDiagnostic.acquire()
        self.diagnostics = msg
        self.semDiagnostic.release()
    def parkLotdetCallbck(self, msg):
        self.semParkLotDet.acquire()
        self.parkLotDet = msg
        self.semParkLotDet.release()
    def desLocInboundCallbck(self, msg):
        self.semDesLocInbound.acquire()
        self.desLocInbound = msg
        self.semDesLocInbound.release()
    def localPoseCallbck(self, msg):
        self.semLocalPose.acquire()
        self.localPose = msg
        self.semLocalPose.release()
    def batteryCallbck(self, msg):
        self.semBattery.acquire()
        self.battery = msg
        self.semBattery.release()
    def relAltCallbck(self, msg):
        self.semRelAlt.acquire()
        self.relAlt = msg
        self.semRelAlt.release()
    def occupancyMapCallbck(self, msg):
        self.semOccupancyMap.acquire()
        self.occupancyMap = msg
        self.semOccupancyMap.release()
    def homePositionCallbck(self, msg):
        self.semHomePosition.acquire()
        self.home_position = msg
        self.semHomePosition.release()
    def getState(self) -> State:
        self.semState.acquire()
        t = self.state
        self.semState.release()
        return t
    def getPose(self) -> NavSatFix:
        self.semGlobalPose.acquire()
        t = self.globalPose
        self.semGlobalPose.release()
        return t
    def getDiagnostics(self):
        # True is normal
        self.semDiagnostic.acquire()
        t = self.diagnostics
        self.semDiagnostic.release()
        return t
    def getParkLotDetected(self):
        self.semParkLotDet.acquire()
        t = self.parkLotDet.data
        self.semParkLotDet.release()
        return t
    def getDesLocInbound(self):
        self.semDesLocInbound.acquire()
        t = self.desLocInbound.data
        self.semDesLocInbound.release()
        return t
    def getHealthStatus(self):
        # True is normal
        # In SITL, the z/altitude control and x/y position control seem to always be false, I dont know why, so I dont know if this is right
        self.semDiagnostic.acquire()
        t = True
        for diag in self.diagnostics.status:
            if diag.level == 2:
                t = False
        self.semDiagnostic.release()
        return t
    def getLocalPose(self) -> PoseStamped:
        self.semLocalPose.acquire()
        t = self.localPose
        self.semLocalPose.release()
        return t
    def getBatteryInfo(self) -> BatteryState:
        self.semBattery.acquire()
        t = self.battery
        self.semBattery.release()
        return t
    def getRelAlt(self):
        self.semRelAlt.acquire()
        t = self.relAlt.data
        self.semRelAlt.release()
        return t
    def getOccupancyMap(self):
        self.semOccupancyMap.acquire()
        t = self.occupancyMap
        self.semOccupancyMap.release()
        return t
    def getHomePosition(self):
        self.semHomePosition.acquire()
        t = self.home_position
        self.semHomePosition.release()
        return t