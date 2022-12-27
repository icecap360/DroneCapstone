import rospy
from geometry_msgs.msg import PoseStamped
from math import pi, sin, cos
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest, ParamSet
from TopicReader import TopicReader
from BasicStates import Idle
import json


class StateMachine:
    def __init__(self, opAppInterface):
        self.nodeName = "fsm_app"
        self.opAppInterface = opAppInterface
        self.paramFile = 'Params.json'
        self.readParams()

    def readParams(self):
        with open(self.paramFile) as json_file:
            data = json.load(json_file)
            self.desiredHoverHeight = data['DesiredHoverHeight']
            self.minHoverHeight = data['MinHoverHeight']
            self.maxHoverHeight = data['MaxHoverHeight']

    def setAndSaveParams(self, minHoverHeight, desiredHoverHeight, maxHoverHeight):
        self.minHoverHeight = minHoverHeight
        self.desiredHoverHeight = desiredHoverHeight
        self.maxHoverHeight = maxHoverHeight
        with open(self.paramFile, 'w') as writer:
            writer.write(json.dumps({
                'MinHoverHeight': self.minHoverHeight, 
                'DesiredHoverHeight': self.desiredHoverHeight, 
                'MaxHoverHeight': self.maxHoverHeight }
                ))

    def process(self):
        self.State = self.State.process()
        self.currStatePub.publish(str(self.State))
        
    def init(self):
        rospy.init_node(self.nodeName)

        self.topicReader = TopicReader()
        #state_sub = rospy.Subscriber("/mavros/state", State, callback = state_cb)
        #global_pose_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, callback = global_pose_cb)
        
        self.currStatePub = rospy.Publisher("/fsm_app/drone_state", String, queue_size=10)
        self.desPosePub = rospy.Publisher("/fsm_desired_pose/desired_pose", PoseStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.armingClient = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.setModeClient = rospy.ServiceProxy("mavros/set_mode", SetMode)
        
        rospy.wait_for_service('/mavros/cmd/takeoff')
        self.takeoffClient = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        rospy.wait_for_service('/mavros/cmd/land')
        self.landClient = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        rospy.wait_for_service('/mavros/param/set')
        self.paramSetClient = rospy.ServiceProxy('/mavros/param/set', ParamSet)

        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)

        self.State = Idle(self,{})

        self.opAppInterface.init()

    def saveHomeLoc(self):
        pose = self.topicReader.getPose()
        self.initHoverPoseGlob = CommandTOL()
        self.initHoverPoseGlob.latitude = pose.latitude
        self.initHoverPoseGlob.longitude = pose.longitude
        self.initHoverPoseGlob.altitude = self.maxHoverHeight

        self.initHoverPose = PoseStamped()
        self.initHoverPose.pose.position.x = 0
        self.initHoverPose.pose.position.y = 0
        self.initHoverPose.pose.position.z = self.maxHoverHeight

    def callService_TypeCommand(self, req, client):
        service_call = client.call(req).success
        while(service_call != True):
            service_call = client.call(req).success
            rospy.sleep(1)
    
    
    # def callService_TypeModeSent(self, req, client):
    #     service_call = client.call(req).mode_sent
    #     while(service_call != True):
    #         service_call = client.call(req).mode_sent
    #         rospy.sleep(0.5)
    def sleep(self, sec:float):
        rospy.sleep(sec)

    def setMode(self, modeReq:str):
        mode = SetModeRequest()
        mode.custom_mode = modeReq
        service_call = self.setModeClient.call(mode).mode_sent
        while(service_call != True):
            service_call = self.setModeClient.call(mode).mode_sent
            rospy.sleep(1.0)
        rospy.loginfo(modeReq + " entered")
        rospy.sleep(2)

    def TestCircularMotion(self):
        rospy.loginfo("initializing...")
        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.topicReader.getState().connected):
            self.rate.sleep()
        
        localPosPub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        rospy.loginfo("Connected...")

        self.saveHomeLoc()

        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            localPosPub.publish(self.initHoverPose)
            self.rate.sleep()

        guided_mode = SetModeRequest()
        guided_mode.custom_mode = 'GUIDED'
        service_call = self.setModeClient.call(guided_mode).mode_sent
        while(service_call != True):
            service_call = self.setModeClient.call(guided_mode).mode_sent
            rospy.sleep(1.0)
        rospy.loginfo("GUIDED enabled")
        rospy.sleep(2)

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        service_call = self.armingClient.call(arm_cmd).success
        while(service_call != True):
            service_call = self.armingClient.call(arm_cmd).success
            rospy.sleep(1.0)
        rospy.loginfo("Vehicle armed")
        
        takeoff_cmd = CommandTOLRequest()
        takeoff_cmd.latitude = self.initHoverPoseGlob.latitude
        takeoff_cmd.longitude = self.initHoverPoseGlob.longitude
        takeoff_cmd.yaw = 0.0
        takeoff_cmd.min_pitch  = 0.0
        takeoff_cmd.altitude = self.initHoverPoseGlob.altitude
        service_call = self.takeoffClient.call(takeoff_cmd).success
        while(service_call != True):
            service_call = self.takeoffClient.call(takeoff_cmd).success
            rospy.sleep(1.0)
        rospy.loginfo("Taking off")
        rospy.sleep(10)

        # Wait while drone is increasing in height 
        while self.topicReader.getPose().altitude < 9.5:
            rospy.sleep(1.0)
        
        # In larger systems, it is often useful to create a new thread which will be in charge of periodically publishing the setpoints.
        time_start = rospy.get_time()
        desPose = PoseStamped()
        desPose.pose.position.z = 10.0
        while(not rospy.is_shutdown()) and rospy.get_time()-time_start<30.0:
            rospy.loginfo("Publishing on localPosPub")
            desPose.pose.position.x = 10*sin(2.0*pi*0.001*(rospy.get_time()-time_start));
            desPose.pose.position.y = 10*cos(2.0*pi*0.001*(rospy.get_time()-time_start));
            desPose.header.stamp = rospy.Time.now()
            localPosPub.publish(desPose)
            self.rate.sleep()

        land_cmd = CommandTOLRequest()
        land_cmd.latitude = self.initHoverPoseGlob.latitude
        land_cmd.longitude = self.initHoverPoseGlob.longitude
        land_cmd.yaw = 0.0
        land_cmd.min_pitch  = 0.0
        land_cmd.altitude = 0
        service_call = self.landClient.call(land_cmd).success
        while(service_call != True):
            service_call = self.landClient.call(land_cmd).success
            rospy.sleep(1.0)
        rospy.loginfo("Landing")
        rospy.spin()
        

if __name__ == '__main__':
    sm = StateMachine(OpAppInterface())
    sm.init()
    sm.process()
    sm.opAppInterface.addCommand({'Type':'Launch','Mode':'Normal'})
    sm.process()