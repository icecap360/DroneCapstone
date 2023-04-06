# Author: Fady
# Date:January  2023
# Purpose: Implementation of path planning and next pose decision

import rospy
from std_msgs.msg import Bool
from MapperApp import MapperApp
from Utils.Common import LogDebug
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GlobalPositionTarget

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
        self.prevPose = None
        print("Pathplan initialized")
    def process(self):
        # plan the path if in autonomous mode (update self.autonomousExplorePose)
        pass
    def publish(self):
        self.topicInterface.pathPlanAppHealth.publish(self.health)
        self.topicInterface.desLocInboundPub.publish(self.getDesLocInBound())
        droneState = self.topicInterface.getDroneState()
        desPoseFSM = self.topicInterface.getDesiredPose()
        # The final localPose should be published by the FSM under all states except Autonomous Explore, within which it is calculated from this module
        if  droneState== 'AutonomousExplore':
            self.topicInterface.localPosPub.publish(self.autonomousExplorePose)
        elif (droneState in [ 'Hover', 'Takeoff', 'DesiredLocationError', 
                'NoParkingLotDetected','CompulsiveMove'] and  # setpointPosPub outside of these states can cause crash
                self.prevPose != desPoseFSM and # only update setpointPosPub upon new desPoseFSM messages
                desPoseFSM.pose.position.latitude != 0.0 and  # lat and long are 0.0 at boot up
                desPoseFSM.pose.position.longitude!=0.0):
            
            #LogDebug('NEXT POSE:\n'+ str(desPoseFSM))
            desPoseFSM.header.stamp = rospy.Time.now()
            self.prevPose = desPoseFSM
            self.topicInterface.setpointPosPub.publish(desPoseFSM)
            

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

