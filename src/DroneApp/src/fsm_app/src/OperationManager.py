# Author: Ali
# Date:December 2022
# Purpose: Main context object that contains all the data needed by the operation states

import rospy
from geometry_msgs.msg import PoseStamped
from math import pi, sin, cos
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest, ParamSet
from TopicInterface import TopicInterface
from ServiceInterface import ServiceInterface
from BasicStates import Idle
import json
from Utils.Common import UserErrorCode, HealthStatusCode, LogDebug
from rospy_message_converter import message_converter

class OperationManager:
    def __init__(self):
        self.nodeName = "fsm_app"
        self.paramFile = 'Params.json'
        self.readParams()
        self.userError = UserErrorCode.NONE
        self.healthStatus = HealthStatusCode.HEALTHY

    def readParams(self):
        # the param file contains the previous height parameters
        with open(self.paramFile) as json_file:
            data = json.load(json_file)
            self.desiredHoverHeight = data['DesiredHoverHeight']
            self.minHoverHeight = data['MinHoverHeight']
            self.maxHoverHeight = data['MaxHoverHeight']

    def setAndSaveParams(self, minHoverHeight, desiredHoverHeight, maxHoverHeight):
        # saves the height parameters in param file
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
        self.FSMState.process()
        self.FSMState = self.FSMState.transitionNextState()
        if self.opAppInterface.isConnected():
            self.sendHeartbeat()
        self.topicInterface.currStatePub.publish(str(self.FSMState))
        
    def init(self, opAppInterface, topicInterf, servInterf):
        rospy.init_node(self.nodeName, disable_signals=True)

        self.topicInterface = topicInterf
        self.servInterface = servInterf
        self.opAppInterface = opAppInterface

        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(10)

        self.FSMState = Idle(self,{})

        self.opAppInterface.init()
    def sendHeartbeat(self):
        homePose = self.topicInterface.getHomePosition()
        localPose = self.topicInterface.getLocalPose()
        globalPose = self.topicInterface.getPose()
        self.opAppInterface.sendMessageAsync( {
            'Type':'Heartbeat',
            'State':str(self.FSMState),
            'ArduMode':self.topicInterface.getState().mode,
            'OccMap':message_converter.convert_ros_message_to_dictionary(
                self.topicInterface.getOccupancyMap()
            ),
            'RelAlt':self.topicInterface.getRelAlt(),
            'Arm':self.topicInterface.getState().armed,
            'Lat':globalPose.latitude,
            'Long':globalPose.longitude,
            'RelX':localPose.pose.position.x,
            'RelY':localPose.pose.position.y,
            'HomeLat':homePose.geo.latitude,
            'HomeLong':homePose.geo.longitude,
            'BattPerc':self.topicInterface.getBatteryInfo().percentage,
            'UserErr':self.userError.value,
            'HealthStatus':self.healthStatus.value
        })
    def saveHomeLoc(self):
        # saves the home location in global and local pose

        pose = self.topicInterface.getPose()
        self.homeHoverPoseGlob = CommandTOL()
        self.homeHoverPoseGlob.latitude = pose.latitude
        self.homeHoverPoseGlob.longitude = pose.longitude
        self.homeHoverPoseGlob.altitude = self.maxHoverHeight

        self.initHoverPose = PoseStamped()
        self.initHoverPose.pose.position.x = 0
        self.initHoverPose.pose.position.y = 0
        self.initHoverPose.pose.position.z = self.maxHoverHeight

    def sleep(self, sec:float):
        rospy.sleep(sec)
    
    def close(self):
        self.opAppInterface.close()
        LogDebug('CLOSING!')

    def spin(self):
        rospy.spin()
        

if __name__ == '__main__':
    sm = OperationManager(OpAppInterface())
    sm.init()
    sm.process()
    sm.opAppInterface.addCommand({'Type':'Launch','Mode':'Normal'})
    sm.process()