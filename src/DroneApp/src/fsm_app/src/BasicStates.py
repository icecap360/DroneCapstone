import rospy
from abc import ABC
from AbstractState import State
from rospy_message_converter import message_converter
import json
from Utils.Common import *
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped


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
        if self.minHoverHeight<7 or self.desiredHoverHeight<7 or self.maxHoverHeight<7:
            self.context.opAppInterface.sendMessageAsync({'Type':'ErrorLog', 'Message':'Height parameters must be atleast 7m'}) 
            self.userError = UserErrorCode.HEIGHT_PARAMS_INVALID
        elif not (self.minHoverHeight<self.desiredHoverHeight and self.desiredHoverHeight<self.maxHoverHeight):
            self.context.opAppInterface.sendMessageAsync({'Type':'ErrorLog', 'Message':'Heaight parameters must be in ascending order: Min,Des,Max'}) 
            self.userError = UserErrorCode.HEIGHT_PARAMS_INVALID
        else:
            self.context.setAndSaveParams(self.minHoverHeight, 
                self.desiredHoverHeight, 
                self.maxHoverHeight)
            self.context.opAppInterface.sendMessageAsync({'Type':'ErrorLog', 'Message':'Heaight parameters successfully updated'}) 
            self.userError = UserErrorCode.NONE
        super().init()
    def evalNewState(self):
        return Idle(self.context, {})

class Idle(State):
    def __init__(self,context, command,name='Idle'):
        super().__init__(context, command, name)
    def init(self):
        super().init()
    def evalNewState(self):
        if not self.context.opAppInterface.isConnected():
            self.context.healthStatus = HealthStatusCode.COMMUNICATION_LOST
            return None
        else:
            self.context.healthStatus = HealthStatusCode.HEALTHY
        command = self.context.opAppInterface.getMessage()
        if (command != None and command['Type'] == 'Arm' and
                self.context.topicInterface.getBatteryInfo().percentage > 0.2):
            return Arm(self.context,{})
        elif command != None and command['Type'] == 'Configure':
            return Configure(self.context, command)
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
        self.context.opAppInterface.sendMessageAsync({'Type':'ErrorLog', 'Message':'Malfunction, system error detected'})
        super().init()
    def exit(self):
        self.context.servInterface.setArm(False)
        rospy.loginfo("Vehicle disarmed")
    def evalNewState(self):
        if not self.context.topicInterface.getState().armed:
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
        elif not self.context.topicInterface.getState().armed:
            return Idle(self.context, {})
        return None

class Hover(FlightState):
    def __init__(self,context, command,name='Hover'):
        super().__init__(context, command, name)
    def init(self):
        self.context.servInterface.setMode('GUIDED')
        #self.context.topicInterface.desPosePub.publish(self.context.topicInterface.getLocalPose())
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
        self.context.opAppInterface.sendMessageAsync({'Type':'ErrorLog', 'Message':'Desired location not within a parking lot'})
        super().init()
    def exit(self):
        self.context.userError = UserErrorCode.NONE

class NoParkingLotDetected(Hover): # reusing code by inheiritng from Hover
    def __init__(self,context, command,name='NoParkingLotDetected'):
        super().__init__(context, command, name)
    def init(self):
        self.context.userError = UserErrorCode.NO_LOT_DETECTED
        self.context.opAppInterface.sendMessageAsync({'Type':'ErrorLog', 'Message':'No parking lot detected'})
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
        if not self.context.topicInterface.getState().armed:
            return Idle(self.context,{})
        return None
    def exit(self):
        self.context.servInterface.setArm(False)
        rospy.loginfo("Vehicle disarmed")

class CompulsiveMove(FlightState):
    def __init__(self,context, command,name='CompulsiveMove'):
        self.lat, self.long, self.W = float(command['Lat']), float(command['Long']), float(command['w'])
        super().__init__(context, command, name)
    def init(self):
        self.context.servInterface.setMode('GUIDED')
        # desPose = PoseStamped()
        # desPose.pose.position.z = self.context.desiredHoverHeight
        # desPose.pose.position.x, desPose.pose.position.y = self.X, self.Y
        # desPose.pose.orientation.w = self.W
        # desPose.header.stamp = rospy.Time.now()
        # self.context.topicInterface.desPosePub.publish(desPose)

        desPose = GeoPoseStamped()
        desPose.pose.position.altitude = self.context.topicInterface.getPose().altitude#
            #self.context.topicInterface.convertToAMSL(
            #self.lat, self.long, self.context.desiredHoverHeight)
        #self.context.topicInterface.getPose().altitude
        desPose.pose.position.latitude, desPose.pose.position.longitude= self.lat, self.long
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

class Arm(State):
    def __init__(self,context, command,name='Arm'):
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
        self.context.servInterface.setArm(True)
        rospy.loginfo("Vehicle armed")
        rospy.sleep(2)
        super().init()
    def during(self):
        if not self.context.topicInterface.getState().armed:
            rospy.loginfo("Vehicle disarmed, rearming")
            self.context.servInterface.setArm(True)
            rospy.loginfo("Vehicle armed")
            rospy.sleep(2)
    def evalNewState(self):
        if not self.context.opAppInterface.isConnected():
            return CommunicationLost(self.context, {})
        #if not self.context.topicInterface.getHealthStatus():
        #    return Malfunction(self.context)
        command = self.context.opAppInterface.getMessage()
        if command != None and command['Type'] == 'Takeoff':
            return Takeoff(self.context,{})
        elif command != None and command['Type'] == 'Disarm':
            self.context.servInterface.setArm(False)
            return Idle(self.context, {})
        else:
            return None

class Takeoff(State):
    def __init__(self,context, command,name='Takeoff'):
        super().__init__(context, command, name)
    def init(self):
        if self.context.topicInterface.getRelAlt() < 5:
            self.context.servInterface.sendTakeoffCmd(self.context.homeHoverPoseGlob)
        rospy.loginfo("Taking off")
        rospy.sleep(8)
        super().init()
    def evalNewState(self):
        if not self.context.opAppInterface.isConnected():
            return CommunicationLost(self.context, {})
        #if not self.context.topicInterface.getHealthStatus():
        #    return Malfunction(self.context)
        if not self.context.topicInterface.getState().armed:
            return Arm(self.context, {})
        elif self.context.topicInterface.getRelAlt() > self.context.homeHoverPoseGlob.altitude - 0.5:
            return Hover(self.context,{})
        else:
            return None