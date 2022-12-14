import rospy
from abc import ABC
from AbstractState import State
from rospy_message_converter import message_converter
import json
from Utils.Common import *
from geometry_msgs.msg import PoseStamped

class FlightState(State):
    # abstract state, created to prevent code duplication for flight states
    def evalNewState(self):
        if not self.context.opAppInterface.isConnected():
            return CommunicationLost(self.context, {})
        if self.context.topicInterface.getBatteryInfo().percentage < 0.2:
            return Land(self.context, {})
        command = self.context.opAppInterface.getMessage()
        if command != None:
            if command['Type'] == 'Land':
                return Land(self.context,{})
            elif command['Type'] == 'CompulsiveMove':
                return CompulsiveMove(self.context, command)
            elif command['Type'] == 'AutonomousMove':
                return AutonomousMove(self.context, command)
            elif command['Type'] == 'AutonomousExplore':
                return AutonomousExplore(self.context, command)
            else:
                return None
        else:
            return None 

class Configure(State):
    def __init__(self,context, command,name='Configure'):
        self.minHoverHeight = float(command['MinHoverHeight'])
        self.desiredHoverHeight = float(command['DesiredHoverHeight'])
        self.maxHoverHeight = float(command['MaxHoverHeight'])
        super().__init__(context,command, name)
    def init(self):
        self.context.setAndSaveParams(self.minHoverHeight, 
            self.desiredHoverHeight, 
            self.maxHoverHeight)
        super().init()
    def evalNewState(self):
        # this is a permanent state
        return None

class Idle(State):
    def __init__(self,context, command,name='Idle'):
        super().__init__(context, command, name)
    def init(self):
        super().init()
    def evalNewState(self):
        print(self.context.opAppInterface.isConnected())
        if not self.context.opAppInterface.isConnected():
            return None
        command = self.context.opAppInterface.getMessage()
        if command != None:
            if (command['Type'] == 'Launch' and command['Mode'] != 'Configure' and 
                    self.context.topicInterface.getBatteryInfo().percentage > 0.2):
                return Takeoff(self.context,{})
            elif command['Type'] == 'Launch' and command['Mode'] == 'Configure':
                return Configure(self.context, command)
            else:
                return None
        else:
            return None

class Malfunction(State):
    def __init__(self, context, command, name='Malfunction'):
        super().__init__(context, command, name)
    def init(self):
        self.context.servInterface.setRtlAlt(700)
        self.context.servInterface.setMode('RTL')
        data_dict = message_converter.convert_ros_message_to_dictionary(self.context.topicInterface.getDiagnostics())
        with open('Logs/ErrorDiagnostic.txt', 'w') as writer:
            writer.write(str(data_dict))
        LogError('MALFUNCTION:\n'+str(data_dict))
        super().init()
    def exit(self):
        self.context.servInterface.setArm(False)
        rospy.loginfo("Vehicle disarmed")
    def evalNewState(self):
        if self.context.topicInterface.getRelAlt() < 0.2:
            return Idle(self.context, {})
        return None

class CommunicationLost(State):
    def __init__(self, context, command, name='CommunicationLost'):
        super().__init__(context, command, name)
    def init(self):
        self.context.healthStatus = HealthStatusCode.COMMUNICATION_LOST
        self.context.servInterface.setRtlAlt(700)
        self.context.servInterface.setMode('RTL')
        super().init()
    def exit(self):
        if self.context.opAppInterface.isConnected():
            # Drone has regained connection
            self.context.healthStatus = HealthStatusCode.HEALTHY
        else:
            # Drone is landing
            self.context.servInterface.setArm(False)
            rospy.loginfo("Vehicle disarmed")
    def evalNewState(self):
        if self.context.opAppInterface.isConnected():
            return Hover(self.context, {})
        elif self.context.topicInterface.getRelAlt() < 0.2:
            return Idle(self.context, {})
        return None

class Hover(FlightState):
    def __init__(self,context, command,name='Hover'):
        super().__init__(context, command, name)
    def init(self):
        self.context.servInterface.setMode('GUIDED')
        self.context.topicInterface.desPosePub.publish(self.context.topicInterface.getLocalPose())
        super().init()
    def evalNewState(self):
        #if not self.context.topicInterface.getHealthStatus():
        #    return Malfunction(self.context)
        return super().evalNewState()

class DesiredLocationError(Hover): # reusing code by inheiritng from Hover
    def __init__(self,context, command,name='DesiredLocationError'):
        super().__init__(context, command, name)
    def init(self):
        self.context.userError = UserErrorCode.DESIRED_LOCATION_INVALID
        super().init()
    def exit(self):
        self.context.userError = UserErrorCode.NONE

class NoParkingLotDetected(Hover): # reusing code by inheiritng from Hover
    def __init__(self,context, command,name='NoParkingLotDetected'):
        super().__init__(context, command, name)
    def init(self):
        self.context.userError = UserErrorCode.NO_LOT_DETECTED
        super().init()
    def exit(self):
        self.context.userError = UserErrorCode.NONE

class AutonomousExplore(FlightState):
    def __init__(self,context, command,name='AutonomousExplore'):
        super().__init__(context, command, name)
    def init(self):
        self.context.servInterface.setMode('GUIDED')
        super().init()
    def evalNewState(self):
        #if not self.context.topicInterface.getHealthStatus():
        #    return Malfunction(self.context)
        if not self.context.topicInterface.getParkLotDetected():
            return NoParkingLotDetected(self.context, {})
        return super().evalNewState()

class Land(State):
    def __init__(self,context, command,name='Land'):
        super().__init__(context, command, name)
    def init(self):
        self.context.servInterface.setRtlAlt(700)
        self.context.servInterface.setMode('RTL')
        super().init()
    def evalNewState(self):
        if self.context.topicInterface.getRelAlt() < 0.2:
            return Idle(self.context,{})
        return None
    def exit(self):
        self.context.servInterface.setArm(False)
        rospy.loginfo("Vehicle disarmed")

class CompulsiveMove(FlightState):
    def __init__(self,context, command,name='CompulsiveMove'):
        self.X, self.Y, self.W = float(command['X']), float(command['Y']), float(command['w'])
        super().__init__(context, command, name)
    def init(self):
        self.context.servInterface.setMode('GUIDED')
        desPose = PoseStamped()
        desPose.pose.position.z = self.context.desiredHoverHeight
        desPose.pose.position.x, desPose.pose.position.y = self.X, self.Y
        desPose.pose.orientation.w = self.W
        desPose.header.stamp = rospy.Time.now()
        self.context.topicInterface.desPosePub.publish(desPose)
        super().init()
    def evalNewState(self):
        #if not self.context.topicInterface.getHealthStatus():
        #    return Malfunction(self.context)
        return super().evalNewState()

class AutonomousMove(FlightState):
    def __init__(self,context, command,name='AutonomousMove'):
        self.X, self.Y, self.W = float(command['X']), float(command['Y']), float(command['w'])
        super().__init__(context, command, name)
    def init(self):
        self.context.servInterface.setMode('GUIDED')
        desPose = PoseStamped()
        desPose.pose.position.z = self.context.desiredHoverHeight
        desPose.pose.position.x, desPose.pose.position.y = self.X, self.Y
        desPose.pose.orientation.w = self.W
        desPose.header.stamp = rospy.Time.now()
        self.context.topicInterface.desPosePub.publish(desPose)
        super().init()
    def evalNewState(self):
        #if not self.context.topicInterface.getHealthStatus():
        #    return Malfunction(self.context)
        if not self.context.topicInterface.getDesLocInbound():
            return DesiredLocationError(self.context, {})
        return super().evalNewState()

class Takeoff(State):
    def __init__(self,context, command,name='Takeoff'):
        super().__init__(context, command, name)
    def init(self):
        rospy.loginfo("initializing...")
        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.context.topicInterface.getState().connected):
            self.context.rate.sleep()

        rospy.loginfo("Connected...")
        self.context.saveHomeLoc()
        self.context.servInterface.setMode('GUIDED')
        rospy.sleep(4)
        super().init()

    def during(self):
        if not self.context.topicInterface.getState().armed:
            self.context.servInterface.setArm(True)
            rospy.loginfo("Vehicle armed")
            rospy.sleep(2)
            self.context.servInterface.sendTakeoffCmd(self.context.homeHoverPoseGlob)
            rospy.loginfo("Taking off")
            rospy.sleep(10)
    
    def evalNewState(self):
        if not self.context.opAppInterface.isConnected():
            return CommunicationLost(self.context, {})
        #if not self.context.topicInterface.getHealthStatus():
        #    return Malfunction(self.context)
        if self.context.topicInterface.getRelAlt() < self.context.homeHoverPoseGlob.altitude - 0.5:
            return None
        else:
            return Hover(self.context,{})