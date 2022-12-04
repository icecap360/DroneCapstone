#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from math import pi, sin, cos
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from ArduInfoReader import ArduInfoReader

class StateMachine:
    def __init__(self):
        self.NodeName = "DroneApp"

    def SetupRosNode(self):
        rospy.init_node(self.NodeName)

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

        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)

        pose = self.arduInfoReader.getPose()
        self.initHoverPoseGlob = CommandTOL()
        self.initHoverPoseGlob.latitude = pose.latitude
        self.initHoverPoseGlob.longitude = pose.longitude
        self.initHoverPoseGlob.altitude = 10

        self.initHoverPose = PoseStamped()
        self.initHoverPose.pose.position.x = 0
        self.initHoverPose.pose.position.y = 0
        self.initHoverPose.pose.position.z = 10


    def Main(self):
        rospy.loginfo("Initializing...")
        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.arduInfoReader.getState().connected):
            self.rate.sleep()

        rospy.loginfo("Connected...")

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
        while(not rospy.is_shutdown()):
            rospy.loginfo("Publishing on localPosPub")
            desPose.pose.position.x = 20*sin(2.0*pi*0.001*(rospy.get_time()-time_start));
            desPose.pose.position.y = 20*cos(2.0*pi*0.001*(rospy.get_time()-time_start));
            desPose.header.stamp = rospy.Time.now()
            self.localPosPub.publish(desPose)
            self.rate.sleep()

if __name__ == "__main__":
    stateMachine = StateMachine()
    stateMachine.SetupRosNode()
    stateMachine.Main()
    
