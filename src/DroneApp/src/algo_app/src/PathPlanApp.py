import rospy
from std_msgs.msg import Bool
from MapperApp import MapperApp
from Utils.Common import LogMessage
from geometry_msgs.msg import PoseStamped

class PathPlanApp:
    def __init__(self):
        self.health = True
        self.desLocInbound = True
        self.autonomousExplorePose = PoseStamped()
        self.prevDroneState = ''
        self.prevDesPoseFSM = PoseStamped()
    def init(self, mapperApp, topicInterface):
        self.mapperApp = mapperApp
        self.topicInterface = topicInterface
        #self.camera.init()
    def process(self, droneState, desLoc):
        # plan the path if in compulsive move (update desired self.desLocInbound)
        # plan the path if in autonomous mode (update self.autonomousExplorePose)
        pass
    def publish(self):
        self.topicInterface.pathPlanAppHealth.publish(self.health)
        self.topicInterface.desLocInboundPub.publish(self.getDesLocInBound())
        droneState = self.topicInterface.getDroneState()
        desPoseFSM = self.topicInterface.getDesiredPose()
        #localPose should be controlled by the FSM under all states except Autonomous Explore, within which it is calculated from this module
        if  droneState== 'AutonomousExplore':
            self.topicInterface.localPosPub.publish(self.autonomousExplorePose)
        elif (droneState in ['Hover', 'DesiredLocationError', 
                'NoParkingLotDetected','CompulsiveMove','AutonomousMove'] and 
                desPoseFSM != localPose):
            localPose = desPoseFSM
            LogMessage('NEXT POSE:\n'+ str(localPose))
            self.topicInterface.localPosPub.publish(localPose)
        
        # Reset variables when error state is exited
        if self.prevDroneState != droneState:
            if self.prevDroneState == 'DesiredLocationError':
                self.desLocInbound = True
                self.topicInterface.desLocInboundPub.publish(self.desLocInbound)
            # if self.prevDroneState == 'NoParkingLotDetected':
            #     self.desLocInbound = True
            #     self.topicInterface.desLocInboundPub.publish(self.getDesLocInBound())
        
        self.prevDroneState = droneState
    def getDesLocInBound(self):
        return self.desLocInbound
    def getHealth(self):
        return self.health

