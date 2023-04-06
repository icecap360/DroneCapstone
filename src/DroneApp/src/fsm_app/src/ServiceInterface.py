# Author: Zaid
# Date: November 2022
# Purpose: Service Interface for DDC modules

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest, ParamSet, ParamSetRequest     

class ServiceInterface:
    def __init__(self):
        rospy.wait_for_service("/mavros/cmd/arming")
        self.armingClient = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.setModeClient = rospy.ServiceProxy("mavros/set_mode", SetMode)
        
        rospy.wait_for_service('/mavros/cmd/takeoff')
        self.takeoffClient = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        rospy.wait_for_service('/mavros/param/set')
        self.paramSetClient = rospy.ServiceProxy('/mavros/param/set', ParamSet)
    
    def callService_TypeCommand(self, req, client):
        # internal helper routine for other services
        service_call = client.call(req).success
        while(service_call != True):
            service_call = client.call(req).success
            rospy.sleep(1)

    def setArm(self, arm:bool):
        armCmd = CommandBoolRequest()
        armCmd.value = arm
        self.callService_TypeCommand(armCmd, self.armingClient)

    def setRtlAlt(self, altitude:int):
        rtlAltCmd = ParamSetRequest()
        rtlAltCmd.param_id = 'RTL_ALT'
        rtlAltCmd.value.integer = altitude 
        rtlAltCmd.value.real = 0.0
        self.callService_TypeCommand(rtlAltCmd, self.paramSetClient)
    
    def sendTakeoffCmd(self, pose):
        takeoffCmd = CommandTOLRequest()
        takeoffCmd.latitude = pose.latitude
        takeoffCmd.longitude = pose.longitude
        takeoffCmd.yaw = 0.0
        takeoffCmd.min_pitch  = 0.0
        takeoffCmd.altitude = pose.altitude
        serviceCall = self.takeoffClient.call(takeoffCmd).success
        while(serviceCall != True):
            serviceCall = self.takeoffClient.call(takeoffCmd).success
            rospy.sleep(1.0)

    def setMode(self, modeReq:str):
        mode = SetModeRequest()
        mode.custom_mode = modeReq
        serviceCall = self.setModeClient.call(mode).mode_sent
        while(serviceCall != True):
            serviceCall = self.setModeClient.call(mode).mode_sent
            rospy.sleep(1.0)
        rospy.loginfo(modeReq + " entered")
        rospy.sleep(2)
    