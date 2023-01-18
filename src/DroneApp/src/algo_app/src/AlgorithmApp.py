import rospy
from threading import Semaphore
from TopicInterface import TopicInterface
from VisionApp import VisionApp
from MapperApp import MapperApp
from PathPlanApp import PathPlanApp
from Utils.Common import LogMessage
from geometry_msgs.msg import PoseStamped

class AlgorithmApp:
    def __init__(self):
        self.desLocInbound = True
        self.droneState = ''
        self.prevDroneState = ''
        self.desPoseFSM = PoseStamped()
        self.prevDesPoseFSM = PoseStamped()
        self.localPose = PoseStamped()
        self.autonomousExplorePose = PoseStamped()
        self.visionApp = VisionApp()
        self.mapperApp = MapperApp(self.visionApp)
        self.pathPlanApp = PathPlanApp(self.mapperApp)

    def init(self):
        rospy.init_node('vision_app', anonymous=True)
        self.topicInterface = TopicInterface()
        # create segmented image publisher
        # create occupancy map publisher
        self.rate = rospy.Rate(10) 
        self.visionApp.init()
        self.mapperApp.init()
        self.pathPlanApp.init()
    
    def pathplan(self):
        pass
    def process(self):
        # Get Inputs
        #self.visionApp.readCamera()
        self.droneState = self.topicInterface.getDroneState()
        self.desPoseFSM = self.topicInterface.getDesiredPose()

        # Run the algorithms
        self.visionApp.process()
        self.mapperApp.process()
        self.pathPlanApp.process(self.droneState)

        # Publish relevent data
        # publish segmented image 
        self.topicInterface.occupancyMapPub.publish(self.mapperApp.getOccupancyMap())
        self.topicInterface.parkLotDetectedPub.publish(self.visionApp.parkLotDetected) #always publish this, as a default transition depends on this
        self.topicInterface.desLocInboundPub.publish(self.desLocInbound)

        #localPose should be controlled by the FSM under all states except Autonomous Explore, within which it is calculated from this module
        if  self.droneState == 'AutonomousExplore':
            self.localPose = self.autonomousExplorePose
        elif self.desPoseFSM != self.localPose:
            self.localPose = self.desPoseFSM
        self.topicInterface.localPosPub.publish(self.localPose)
        
        # reset variables when error state is exited
        if self.prevDroneState != self.droneState:
            if self.prevDroneState == 'DesiredLocationError':
                self.desLocInbound = True
                self.topicInterface.desLocInboundPub.publish(self.desLocInbound)
            # if self.prevDroneState == 'NoParkingLotDetected':
            #     self.desLocInbound = True
            #     self.topicInterface.desLocInboundPub.publish(self.desLocInbound)
        
        self.prevDroneState = self.droneState 
        self.prevDesPoseFSM = self.desPoseFSM        
        self.rate.sleep()