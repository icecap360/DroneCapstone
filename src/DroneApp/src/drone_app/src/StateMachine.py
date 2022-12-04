import rospy
from geometry_msgs.msg import PoseStamped
from math import pi, sin, cos
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from ArduInfoReader import ArduInfoReader
from BasicStates import Idle
from OpAppInterface import OpAppInterface
import json

class StateMachine:
    def __init__(self, opAppInterface: OpAppInterface):
        self.nodeName = "DroneApp"
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
        self.State = self.State.Process()
        
    def SetupRosNode(self):
        rospy.init_node(self.nodeName)

        self.arduInfoReader = ArduInfoReader()
        #state_sub = rospy.Subscriber("/mavros/state", State, callback = state_cb)
        #global_pose_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, callback = global_pose_cb)

        self.localPosPub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        rospy.wait_for_service("/mavros/cmd/arming")
        self.armingClient = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.setModeClient = rospy.ServiceProxy("mavros/set_mode", SetMode)
        
        rospy.wait_for_service('/mavros/cmd/takeoff')
        self.takeoffClient = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        rospy.wait_for_service('//mavros/cmd/land')
        self.landClient = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)

        self.State = Idle(self)

    def saveHomeLoc(self):
        pose = self.arduInfoReader.getPose()
        self.initHoverPoseGlob = CommandTOL()
        self.initHoverPoseGlob.latitude = pose.latitude
        self.initHoverPoseGlob.longitude = pose.longitude
        self.initHoverPoseGlob.altitude = self.maxHoverHeight

        self.initHoverPose = PoseStamped()
        self.initHoverPose.pose.position.x = 0
        self.initHoverPose.pose.position.y = 0
        self.initHoverPose.pose.position.z = self.maxHoverHeight

    def TestCircularMotion(self):
        rospy.loginfo("Initializing...")
        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.arduInfoReader.getState().connected):
            self.rate.sleep()

        rospy.loginfo("Connected...")

        self.saveHomeLoc()

        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            self.localPosPub.publish(self.initHoverPose)
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
        while self.arduInfoReader.getPose().altitude < 9.5:
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
            self.localPosPub.publish(desPose)
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
    print('a')
    sm.SetupRosNode()
    print('b')
    sm.process()
    print('c')
    sm.opAppInterface.addCommand({'Type':'Launch','Mode':'Normal'})
    print('d')
    sm.process()