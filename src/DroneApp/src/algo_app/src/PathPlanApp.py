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
        # plan the path if in compulsive move (update desired self.desLocInbound)
        # plan the path if in autonomous mode (update self.autonomousExplorePose)
        pass
    def publish(self):
        self.topicInterface.pathPlanAppHealth.publish(self.health)
        self.topicInterface.desLocInboundPub.publish(self.getDesLocInBound())
        droneState = self.topicInterface.getDroneState()
        desPoseFSM = self.topicInterface.getDesiredPose()
        localPose = self.topicInterface.getLocalPose()
        # The final localPose should be published by the FSM under all states except Autonomous Explore, within which it is calculated from this module
        if  droneState== 'AutonomousExplore':
            self.topicInterface.localPosPub.publish(self.autonomousExplorePose)
        elif (droneState in [ 'Hover', 'Takeoff', 'DesiredLocationError', 
                'NoParkingLotDetected','CompulsiveMove'] and 
                self.prevPose != desPoseFSM and
                desPoseFSM.pose.position.latitude != 0.0 and 
                desPoseFSM.pose.position.longitude!=0.0):
            
            LogDebug('NEXT POSE:\n'+ str(desPoseFSM))
            
            #self.topicInterface.localPosPub.publish(localPose)
            
            # localPose = desPoseFSM
            desPoseFSM.header.stamp = rospy.Time.now()
            self.prevPose = desPoseFSM
            self.topicInterface.globalPosPub.publish(desPoseFSM)
            
            # desPose = GlobalPositionTarget()
            # desPose.altitude = desPoseFSM.pose.position.altitude
            # desPose.latitude, desPose.longitude= desPoseFSM.pose.position.latitude, desPoseFSM.pose.position.longitude
            # desPose.type_mask = 1024 & 2048
            # desPose.yaw = desPoseFSM.pose.orientation.w
            # desPose.header.stamp = rospy.Time.now()
            # self.topicInterface.globalPosPubRaw.publish(desPose)

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

