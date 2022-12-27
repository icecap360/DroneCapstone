import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from threading import Semaphore
from TopicReader import TopicReader
from VisionApp import VisionApp
from Utils.Common import LogMessage

class PathPlanApp:
    def __init__(self):
        self.visionApp = VisionApp()
        self.desLocInbound = True
        self.droneState = ''
        self.prevDroneState = ''
        self.desPoseFSM = PoseStamped()
        self.prevDesPoseFSM = PoseStamped()
        self.localPose = PoseStamped()
        self.autonomousExplorePose = PoseStamped()

    def init(self):
        rospy.init_node('vision_app', anonymous=True)
        self.topicReader = TopicReader()
        self.parkLotDetectedPub = rospy.Publisher('/path_plan_app/parking_lot_detected', Bool, queue_size=10)
        self.parkLotDetectedPub.publish(False) #initialize to Talse
        self.desLocInboundPub = rospy.Publisher('/path_plan_app/desired_loc_inbound', Bool, queue_size=10)
        self.desLocInboundPub.publish(True) # initialize to True
        self.localPosPub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        # create segmented image publisher
        # create occupancy map publisher
        self.rate = rospy.Rate(10) 
        self.visionApp.init()
    
    def pathplan(self):
        # plan the path if in compulsive move (update desired self.desLocInbound)
        # plan the path if in autonomous mode (update self.autonomousExplorePose)
        # update occupancy map
        pass
    def process(self):
        # Get Inputs
        #self.visionApp.readCamera()
        self.droneState = self.topicReader.getDroneState()
        self.desPoseFSM = self.topicReader.getDesiredPose()

        # Run the algorithms
        self.visionApp.process()
        self.pathplan()

        # Publish relevent data:
        # publish segmented image 
        # publish occupancy map
        self.parkLotDetectedPub.publish(self.visionApp.parkLotDetected) #always publish this, as a default transition depnds on this
        self.desLocInboundPub.publish(self.desLocInbound)

        #localPose should be controlled by the FSM under all states except Autonomous Explore, within which it is calculated from this module
        if  self.droneState == 'AutonomousExplore':
            self.localPose = self.autonomousExplorePose
        elif self.desPoseFSM != self.localPose:
            self.localPose = self.desPoseFSM
        self.localPosPub.publish(self.localPose)
        
        # reset variables when error state is exited
        if self.prevDroneState != self.droneState:
            if self.prevDroneState == 'DesiredLocationError':
                self.desLocInbound = True
                self.desLocInboundPub.publish(self.desLocInbound)
            if self.prevDroneState == 'NoParkingLotDetected':
                self.desLocInbound = True
                self.desLocInboundPub.publish(self.desLocInbound)
        
        self.prevDroneState = self.droneState 
        self.prevDesPoseFSM = self.desPoseFSM        
        self.rate.sleep()