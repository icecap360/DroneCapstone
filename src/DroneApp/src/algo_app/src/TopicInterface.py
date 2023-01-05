import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from threading import Semaphore
from algo_app.msg import OccupancyMap 
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
class TopicInterface:
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
        self.drone_state = String()
        self.sem_drone_state = Semaphore()
        self.current_state_sub = rospy.Subscriber("/fsm_app/drone_state", String, callback = self.drone_state_cb)

        self.des_pose = PoseStamped()
        self.sem_des_pose = Semaphore()
        self.des_pose_sub = rospy.Subscriber("/fsm_desired_pose/desired_pose", PoseStamped, callback = self.des_pose_cb)

        self.local_pose = PoseStamped()
        self.sem_local_pose = Semaphore()
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = self.local_pose_cb)
        
        self.parkLotDetectedPub = rospy.Publisher('/algo_app/parking_lot_detected', Bool, queue_size=10)
        self.parkLotDetectedPub.publish(False) #initialize to Talse
        self.desLocInboundPub = rospy.Publisher('/algo_app/desired_loc_inbound', Bool, queue_size=10)
        self.desLocInboundPub.publish(True) # initialize to True
        self.localPosPub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.occupancyMapPub = rospy.Publisher("/algo_app/occupancy_map", OccupancyMap, queue_size=10)

    def drone_state_cb(self, msg):
        self.sem_drone_state.acquire()
        self.drone_state = msg.data
        self.sem_drone_state.release()
    def des_pose_cb(self, msg):
        self.sem_des_pose.acquire()
        self.des_pose = msg
        self.sem_des_pose.release()
    def local_pose_cb(self, msg):
        self.sem_local_pose.acquire()
        self.local_pose = msg
        self.sem_local_pose.release()

    def getDroneState(self):
        self.sem_drone_state.acquire()
        t = self.drone_state
        self.sem_drone_state.release()
        return t
    def getDesiredPose(self):
        self.sem_des_pose.acquire()
        t = self.des_pose
        self.sem_des_pose.release()
        return t
    def getLocalPose(self) -> PoseStamped:
        self.sem_local_pose.acquire()
        t = self.local_pose
        self.sem_local_pose.release()
        return t