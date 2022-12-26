import rospy
from abc import ABC
from AbstractState import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest, ParamSet, ParamSetRequest     
from rospy_message_converter import message_converter
import json
from Utils.Common import *
from geometry_msgs.msg import PoseStamped

class Configure(State):
    def __init__(self,context, command):
        self.minHoverHeight = float(command['MinHoverHeight'])
        self.desiredHoverHeight = float(command['DesiredHoverHeight'])
        self.maxHoverHeight = float(command['MaxHoverHeight'])
        super().__init__(context,command, 'Configure')
    
    def Init(self) -> State:
        self.context.setAndSaveParams(self.minHoverHeight, 
            self.desiredHoverHeight, 
            self.maxHoverHeight)
        return super().Init()
    
    def During(self) -> State:
        rospy.loginfo('launch during')
        pass

class Idle(State):
    def __init__(self,context, command):
        super().__init__(context, command, 'Idle')
    
    def Init(self) -> State:
        return super().Init()
    
    def During(self) -> State:        
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
    def __init__(self, context, command):
        super().__init__(context, command, 'Malfunction')
    
    def Init(self) -> State:
        rtl_alt_cmd = ParamSetRequest()
        rtl_alt_cmd.param_id = 'RTL_ALT'
        rtl_alt_cmd.value.integer = 700 
        rtl_alt_cmd.value.real = 0.0
        self.context.callService_TypeCommand(rtl_alt_cmd, self.context.paramSetClient)
        self.context.setMode('RTL')

        data_dict = message_converter.convert_ros_message_to_dictionary(self.context.arduInfoReader.getDiagnostics())
        with open('Logs\\ErrorDiagnostic.txt', 'w') as writer:
            writer.write(str(data_dict))
        LogError('MALFUNCTION:\n'+str(data_dict))
        return super().Init()
    
    def During(self) -> State:
        if self.context.arduInfoReader.getPose().altitude < 0.2:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = False
            self.context.callService_TypeCommand(arm_cmd, self.context.armingClient)
            rospy.loginfo("Vehicle disarmed")
        return self   

class Hover(State):
    def __init__(self,context, command):
        super().__init__(context, command, 'Hover')
    
    def Init(self) -> State:
        self.context.setMode('GUIDED')
        return super().Init()
    
    def During(self) -> State:
        #if not self.context.arduInfoReader.getHealthStatus():
        #    return Malfunction(self.context)

        command = self.context.opAppInterface.getMessage()
        if command != None:
            if command['Type'] == 'Land':
                return Land(self.context,{})
            elif command['Type'] == 'CompulsiveMove':
                return CompulsiveMove(self.context, command)
            else:
                return self
        else:
            return self   

class Land(State):
    def __init__(self,context, command):
        super().__init__(context, command, 'Land')
    
    def Init(self) -> State:
        rtl_alt_cmd = ParamSetRequest()
        rtl_alt_cmd.param_id = 'RTL_ALT'
        rtl_alt_cmd.value.integer = 700 
        rtl_alt_cmd.value.real = 0.0
        self.context.callService_TypeCommand(rtl_alt_cmd, self.context.paramSetClient)
        self.context.setMode('RTL')
        return super().Init()
    
    def During(self) -> State:        
        if self.context.arduInfoReader.getPose().altitude > 0.2:
            return self
        else:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = False
            self.context.callService_TypeCommand(arm_cmd, self.context.armingClient)
            rospy.loginfo("Vehicle disarmed")
            return Idle(self.context,{})

class CompulsiveMove(State):
    def __init__(self,context,command):
        self.X = float(command['X'])
        self.Y = float(command['Y'])
        self.w = float(command['w'])
        super().__init__(context, command, 'CompulsiveMove')
    
    def Init(self) -> State:
        self.context.setMode('GUIDED')
        desPose = PoseStamped()
        desPose.pose.position.z = self.context.desiredHoverHeight
        desPose.pose.position.x = self.Y
        desPose.pose.position.y = self.X
        desPose.pose.orientation.w = self.w
        desPose.header.stamp = rospy.Time.now()
        self.context.localPosPub.publish(desPose)
        return super().Init()
    
    def During(self) -> State:
        #if not self.context.arduInfoReader.getHealthStatus():
        #    return Malfunction(self.context)

        command = self.context.opAppInterface.getMessage()
        if command != None:
            if command['Type'] == 'Land':
                return Land(self.context, {})
            elif command['Type'] == 'CompulsiveMove':
                return CompulsiveMove(self.context, command)    
            else:
                return self
        else:
            return self  

class Takeoff(State):
    def __init__(self,context, command):
        super().__init__(context, command, 'Takeoff')
    
    def Init(self) -> State:
        rospy.loginfo("Initializing...")
        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.context.arduInfoReader.getState().connected):
            self.context.rate.sleep()

        rospy.loginfo("Connected...")

        self.context.saveHomeLoc()

        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            self.context.localPosPub.publish(self.context.initHoverPose)
            self.context.rate.sleep()

        self.context.setMode('GUIDED')
        rospy.sleep(2)

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

        return super().Init()
    
    def During(self) -> State:
        #if not self.context.arduInfoReader.getHealthStatus():
        #    return Malfunction(self.context)
        
        if self.context.arduInfoReader.getPose().altitude < self.context.initHoverPoseGlob.altitude - 0.5:
            return self
        else:
            return Hover(self.context,{})