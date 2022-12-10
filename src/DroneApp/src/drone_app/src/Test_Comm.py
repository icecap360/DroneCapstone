#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from rospy_message_converter import message_converter
from sensor_msgs.msg import Imu

import sys
sys.path.append('/home/pi/DroneCapstone/src')
import Utils.Sockets as Sockets
from Utils.Common import *

class DroneApp:
	def __init__(self):
		pass
	def init(self):
		self.OpAppInterface = Sockets.DroneSocket()
		self.OpAppInterface.init()
		# In ROS, nodes are uniquely named. If two nodes with the same
		# name are launched, the previous one is kicked off. The
		# anonymous=True flag means that rospy will choose a unique
		# name for our 'listener' node so that multiple listeners can
		# run simultaneously.
		rospy.init_node('MainDroneApp', anonymous=True)
		rospy.Subscriber("/mavros/imu/data", Imu, self.callback)
		LogMessage('Finished initializing')
		
	def spin(self):
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

	def callback(self, data):
		#rospy.loginfo(rospy.get_caller_id() + "Sending: %s", data)
		data_dict = message_converter.convert_ros_message_to_dictionary(data)
		data = self.OpAppInterface.getMessage()
		if data:
			print(data)
		self.OpAppInterface.sendMessageSync(data_dict)

if __name__ == '__main__':
	droneApp = DroneApp()
	droneApp.init()
	droneApp.spin()