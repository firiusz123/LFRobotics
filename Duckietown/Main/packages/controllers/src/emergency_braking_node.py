#!/usr/bin/env python3

import rospy
import numpy as np

# import DTROS-related classes
from duckietown.dtros import \
    DTROS, \
    NodeType, \
    DTParam, \
    ParamType

# import messages and services
from std_msgs.msg import Float32, Header
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, FSMState

class EmergencyBraking(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(EmergencyBraking, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        
        # Setting up parameters
        self.distance = DTParam('~distance', param_type=ParamType.FLOAT)

        # # Subsriber data
        self.subscriber = rospy.Subscriber('~in', Float32, self.callback, queue_size=1)

        # # Construct publisher
        self.pub_control = rospy.Publisher('~cmd', Twist2DStamped, queue_size=1)

    def callback(self, msg) -> None:
        try:
            if msg.data<self.distance.value :
                self.pub_control.publish(Twist2DStamped(omega=0.0, v=0.0))

        except Exception as ex:
            rospy.logerr(ex)

    def on_switch_on(self):
        rospy.loginfo("Emergency breaking node switched from off to on")

    def on_switch_off(self): # The node will shut down before this code is run, so it's effectively useless, but it reminds us of inevitable death upon which we can take no action and after which we will serve no purpose
        pass 

    def on_shutdown(self):
        self.subscriber.unregister()

if __name__ == '__main__':
    # create the node
    node = EmergencyBraking(node_name='emergency_braking_node')
    rospy.spin()