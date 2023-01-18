import rospy
from std_msgs.msg import Bool
from MapperApp import MapperApp
class PathPlanApp:
    def __init__(self, mapper: MapperApp):
        self.map = mapper
        #self.camera = Camera()
    def init(self):
        #self.camera.init()
        pass
    def process(self, droneState):
        # plan the path if in compulsive move (update desired self.desLocInbound)
        # plan the path if in autonomous mode (update self.autonomousExplorePose)
        pass

