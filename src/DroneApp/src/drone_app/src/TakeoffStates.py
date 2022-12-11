import rospy
from AbstractStates import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest


class Takeoff(State):
    def __init__(self,context):
        super().__init__(context, 'Takeoff')
    
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

        # Wait while drone is increasing in height 
        while self.context.arduInfoReader.getPose().altitude < self.context.initHoverPoseGlob.altitude - 0.5:
            rospy.sleep(1.0)
        return super().Init()
    
    def During(self) -> State:
        return Hover(self.context)

class Hover(State):
    def __init__(self,context):
        super().__init__(context, 'Hover')
    
    def Init(self) -> State:
        self.context.setMode('GUIDED')
        return super().Init()
    
    def During(self) -> State:        
        return self