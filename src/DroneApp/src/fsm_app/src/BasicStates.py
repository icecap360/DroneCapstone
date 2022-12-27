import rospy
from abc import ABC
from AbstractState import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest, ParamSet, ParamSetRequest     
from rospy_message_converter import message_converter
import json
from Utils.Common import *
from geometry_msgs.msg import PoseStamped

class FlightState(State):
    # abstract state, created to prevent code duplication for flight states
    def processUserCommand(self, command) -> State:
        # default way to process user commands
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
                return self
        else:
            return self 

class Configure(State):
    def __init__(self,context, command,name='Configure'):
        self.minHoverHeight = float(command['MinHoverHeight'])
        self.desiredHoverHeight = float(command['DesiredHoverHeight'])
        self.maxHoverHeight = float(command['MaxHoverHeight'])
        super().__init__(context,command, name)
    def init(self) -> State:
        self.context.setAndSaveParams(self.minHoverHeight, 
            self.desiredHoverHeight, 
            self.maxHoverHeight)
        return super().init()
    def during(self) -> State:
        rospy.loginfo('launch during')
        pass

class Idle(State):
    def __init__(self,context, command,name='Idle'):
        super().__init__(context, command, name)
    def init(self) -> State:
        return super().init()
    def during(self) -> State:        
        command = self.context.opAppInterface.getMessage()
        if command != None:
            if command['Type'] == 'Launch' and command['Mode'] != 'Configure':
                return Takeoff(self.context,{})
            elif command['Type'] == 'Launch' and command['Mode'] == 'Configure':
                return Configure(self.context, command)
            else:
                return self
        else:
            return self

class Malfunction(State):
    def __init__(self, context, command, name='Malfunction'):
        super().__init__(context, command, name)
    
    def init(self) -> State:
        rtl_alt_cmd = ParamSetRequest()
        rtl_alt_cmd.param_id = 'RTL_ALT'
        rtl_alt_cmd.value.integer = 700 
        rtl_alt_cmd.value.real = 0.0
        self.context.callService_TypeCommand(rtl_alt_cmd, self.context.paramSetClient)
        self.context.setMode('RTL')

        data_dict = message_converter.convert_ros_message_to_dictionary(self.context.topicReader.getDiagnostics())
        with open('Logs\\ErrorDiagnostic.txt', 'w') as writer:
            writer.write(str(data_dict))
        LogError('MALFUNCTION:\n'+str(data_dict))
        return super().init()

    def during(self) -> State:
        if self.context.topicReader.getPose().altitude < 0.2:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = False
            self.context.callService_TypeCommand(arm_cmd, self.context.armingClient)
            rospy.loginfo("Vehicle disarmed")
        return self   

class Hover(FlightState):
    def __init__(self,context, command,name='Hover'):
        super().__init__(context, command, name)
    def init(self) -> State:
        self.context.setMode('GUIDED')
        self.context.desPosePub.publish(self.context.topicReader.getLocalPose())
        return super().init()
    def during(self) -> State:
        #if not self.context.topicReader.getHealthStatus():
        #    return Malfunction(self.context)
        command = self.context.opAppInterface.getMessage()
        return self.processUserCommand(command) 

class DesiredLocationError(Hover): # reusing code by inheiritng from Hover
    def __init__(self,context, command,name='DesiredLocationError'):
        super().__init__(context, command, name)

class NoParkingLotDetected(Hover): # reusing code by inheiritng from Hover
    def __init__(self,context, command,name='NoParkingLotDetected'):
        super().__init__(context, command, name)

class AutonomousExplore(FlightState):
    def __init__(self,context, command,name='AutonomousExplore'):
        super().__init__(context, command, name)
    def init(self) -> State:
        self.context.setMode('GUIDED')
        return super().init()
    def during(self) -> State:
        #if not self.context.topicReader.getHealthStatus():
        #    return Malfunction(self.context)
        if not self.context.topicReader.getParkLotDetected():
            return NoParkingLotDetected(self.context, {})
        command = self.context.opAppInterface.getMessage()
        return self.processUserCommand(command)

class Land(State):
    def __init__(self,context, command,name='Land'):
        super().__init__(context, command, name)
    def init(self) -> State:
        rtl_alt_cmd = ParamSetRequest()
        rtl_alt_cmd.param_id = 'RTL_ALT'
        rtl_alt_cmd.value.integer = 700 
        rtl_alt_cmd.value.real = 0.0
        self.context.callService_TypeCommand(rtl_alt_cmd, self.context.paramSetClient)
        self.context.setMode('RTL')
        return super().init()
    def during(self) -> State:        
        if self.context.topicReader.getPose().altitude > 0.2:
            return self
        else:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = False
            self.context.callService_TypeCommand(arm_cmd, self.context.armingClient)
            rospy.loginfo("Vehicle disarmed")
            return Idle(self.context,{})

class CompulsiveMove(FlightState):
    def __init__(self,context, command,name='CompulsiveMove'):
        self.X, self.Y, self.W = float(command['X']), float(command['Y']), float(command['w'])
        super().__init__(context, command, name)
    def init(self) -> State:
        self.context.setMode('GUIDED')
        desPose = PoseStamped()
        desPose.pose.position.z = self.context.desiredHoverHeight
        desPose.pose.position.x, desPose.pose.position.y = self.X, self.Y
        desPose.pose.orientation.w = self.w
        desPose.header.stamp = rospy.Time.now()
        self.context.desPosePub.publish(desPose)
        return super().init()
    def during(self) -> State:
        #if not self.context.topicReader.getHealthStatus():
        #    return Malfunction(self.context)
        command = self.context.opAppInterface.getMessage()
        return self.processUserCommand(command)

class AutonomousMove(FlightState):
    def __init__(self,context, command,name='AutonomousMove'):
        self.X, self.Y, self.W = float(command['X']), float(command['Y']), float(command['w'])
        super().__init__(context, command, name)
    def init(self) -> State:
        self.context.setMode('GUIDED')
        desPose = PoseStamped()
        desPose.pose.position.z = self.context.desiredHoverHeight
        desPose.pose.position.x, desPose.pose.position.y = self.X, self.Y
        desPose.pose.orientation.w = self.w
        desPose.header.stamp = rospy.Time.now()
        self.context.desPosePub.publish(desPose)
        return super().init()
    def during(self) -> State:
        #if not self.context.topicReader.getHealthStatus():
        #    return Malfunction(self.context)
        if not self.context.topicReader.getDesLocInbound():
            return DesiredLocationError(self.context, {})
        command = self.context.opAppInterface.getMessage()
        return self.processUserCommand(command)

class Takeoff(State):
    def __init__(self,context, command,name='Takeoff'):
        super().__init__(context, command, name)
    
    def init(self) -> State:
        rospy.loginfo("initializing...")
        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.context.topicReader.getState().connected):
            self.context.rate.sleep()

        rospy.loginfo("Connected...")

        self.context.saveHomeLoc()

        self.context.setMode('GUIDED')
        rospy.sleep(4)

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        self.context.callService_TypeCommand(arm_cmd, self.context.armingClient)
        rospy.loginfo("Vehicle armed")
        
        takeoff_cmd = CommandTOLRequest()
        takeoff_cmd.latitude = self.context.initHoverPoseGlob.latitude
        takeoff_cmd.longitude = self.context.initHoverPoseGlob.longitude
        takeoff_cmd.yaw = 0.0
        takeoff_cmd.min_pitch  = 0.0
        takeoff_cmd.altitude = self.context.initHoverPoseGlob.altitude
        self.context.callService_TypeCommand(takeoff_cmd, self.context.takeoffClient)
        rospy.loginfo("Taking off")
        rospy.sleep(10)

        return super().init()
    
    def during(self) -> State:
        #if not self.context.topicReader.getHealthStatus():
        #    return Malfunction(self.context)
        
        if self.context.topicReader.getPose().altitude < self.context.initHoverPoseGlob.altitude - 0.5:
            return self
        else:
            return Hover(self.context,{})