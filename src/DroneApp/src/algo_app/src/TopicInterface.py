# Author: Ali
# Date:November 2022
# Purpose: Module that is used to get and send data from topics

import rospy
from pygeodesy.geoids import GeoidPGM 
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State, GlobalPositionTarget
from threading import Semaphore
from algo_app.msg import OccupancyMap 
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

class TopicInterface:
    '''
    This class is supposed to read from topics that come from the FCU and other nodes. 
    Any algorithm module should store all its topic related behaviour here.

    https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level
    When controlling the FCU using global setpoints, you specify the altitude as 
    meters above mean sea level (AMSL). But when sensing the global position, the 
    altitude reported by ~global_position/global is specified as meters above the 
    WGS-84 ellipsoid. This can lead to differences in altitude that are dozens of 
    meters apart.
    '''
    
    def __init__(self):
        self._egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)
        self.droneState = String()
        self.semDroneState = Semaphore()
        self.currentStateSub = rospy.Subscriber("/fsm_app/drone_state", String, callback = self.droneStateCb)

        self.desPose = GeoPoseStamped()
        self.semDesPose = Semaphore()
        self.desPoseSub = rospy.Subscriber("/fsm_desired_pose/desired_pose", GeoPoseStamped, callback = self.desPosecb)

        self.localPose = PoseStamped()
        self.semLocalPose = Semaphore()
        self.localPoseSub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = self.localPoseCb)

        self.globalPose = NavSatFix()
        self.semGlobalPose = Semaphore()
        self.globalPoseSub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, callback = self.globalPoseCb)

        self.parkLotDetectedPub = rospy.Publisher('/algo_app/parking_lot_detected', Bool, queue_size=10)
        self.parkLotDetectedPub.publish(False) #initialize to Talse
        self.desLocInboundPub = rospy.Publisher('/algo_app/desired_loc_inbound', Bool, queue_size=10)
        self.desLocInboundPub.publish(True) # initialize to True
        #self.localPosPub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.setpointPosPub = rospy.Publisher("/mavros/setpoint_position/global", GeoPoseStamped, queue_size=10)
        self.setpointPosPubRaw = rospy.Publisher("/mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=10)
        self.occupancyMapPub = rospy.Publisher("/algo_app/occupancy_map", OccupancyMap, queue_size=10)
        self.visionAppHealth = rospy.Publisher("/algo_app/vision_app_health", Bool, queue_size=10)
        self.mapperAppHealth = rospy.Publisher("/algo_app/mapper_app_health", Bool, queue_size=10)
        self.pathPlanAppHealth = rospy.Publisher("/algo_app/path_plan_app_health", Bool, queue_size=10)
    def convertToAMSL(self,lat, lon, height) -> float:
        return height - self._egm96.height(lat, lon)
    def convertToEllipsoid(self, lat, lon, height) -> float:
        return height + self._egm96.height(lat, lon)
    def droneStateCb(self, msg):
        self.semDroneState.acquire()
        self.droneState = msg.data
        self.semDroneState.release()
    def desPosecb(self, msg):
        self.semDesPose.acquire()
        self.desPose = msg
        self.semDesPose.release()
    def localPoseCb(self, msg):
        self.semLocalPose.acquire()
        self.localPose = msg
        self.semLocalPose.release()

    def getDroneState(self):
        self.semDroneState.acquire()
        t = self.droneState
        self.semDroneState.release()
        return t
    def getDesiredPose(self):
        self.semDesPose.acquire()
        t = self.desPose
        self.semDesPose.release()
        return t
    def getLocalPose(self) -> PoseStamped:
        self.semLocalPose.acquire()
        t = self.localPose
        self.semLocalPose.release()
        return t
    def globalPoseCb(self, msg):
        self.semGlobalPose.acquire()
        msg.altitude = self.convertToAMSL(msg.latitude, msg.longitude, msg.altitude) 
        self.globalPose = msg
        self.semGlobalPose.release()
    def getGlobalPose(self) -> NavSatFix:
        self.semGlobalPose.acquire()
        t = self.globalPose
        self.semGlobalPose.release()
        return t