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
        service_call = client.call(req).success
        while(service_call != True):
            service_call = client.call(req).success
            rospy.sleep(1)

    def setArm(self, arm:bool):
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = arm
        self.callService_TypeCommand(arm_cmd, self.armingClient)

    def setRtlAlt(self, altitude:int):
        rtl_alt_cmd = ParamSetRequest()
        rtl_alt_cmd.param_id = 'RTL_ALT'
        rtl_alt_cmd.value.integer = altitude 
        rtl_alt_cmd.value.real = 0.0
        self.callService_TypeCommand(rtl_alt_cmd, self.paramSetClient)
    
    def sendTakeoffCmd(self, pose):
        takeoff_cmd = CommandTOLRequest()
        takeoff_cmd.latitude = pose.latitude
        takeoff_cmd.longitude = pose.longitude
        takeoff_cmd.yaw = 0.0
        takeoff_cmd.min_pitch  = 0.0
        takeoff_cmd.altitude = pose.altitude
        service_call = self.takeoffClient.call(takeoff_cmd).success
        while(service_call != True):
            service_call = self.takeoffClient.call(takeoff_cmd).success
            rospy.sleep(1.0)

    def setMode(self, modeReq:str):
        mode = SetModeRequest()
        mode.custom_mode = modeReq
        service_call = self.setModeClient.call(mode).mode_sent
        while(service_call != True):
            service_call = self.setModeClient.call(mode).mode_sent
            rospy.sleep(1.0)
        rospy.loginfo(modeReq + " entered")
        rospy.sleep(2)
    
    # def callService_TypeModeSent(self, req, client):
    #     service_call = client.call(req).mode_sent
    #     while(service_call != True):
    #         service_call = client.call(req).mode_sent
    #         rospy.sleep(0.5)