#!/usr/bin/env python3
import rospy

# import DTROS-related classes
from numpy import rad2deg as rad2deg
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

from std_msgs.msg import Int32

from random import randint

class TurnDetectionNode(DTROS):

    def __init__(self,nodeName="turn_detection_node"):
        super().__init__(nodeName)

        # Parameters to detect the available turn patterns

        self.turn_options_service = rospy.Service("~get_turn_options",Int32,self.random_turn)

    def random_turn():
        return randint(1,4)
        
    def detect_turn_options():
        pass

if __name__ == "__main__":
    turnDetecitonNode = TurnDetectionNode()
