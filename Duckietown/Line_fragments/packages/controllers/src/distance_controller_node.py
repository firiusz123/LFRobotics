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
from std_msgs.msg import Float32
from duckietown_msgs.msg import BoolStamped

class DriveController(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DriveController, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        
        self.msg = BoolStamped()

        # Setting up parameters
        self.distance = DTParam('~distance', param_type=ParamType.DICT)

        # subsriber data
        self.subscriber = rospy.Subscriber('~in', Float32, self.callback, queue_size=1)

        self.topics = {
            'emergency_braking'         : False, 
            'line_follower'             : False, 
            'adaptive_cruise_control'   : False}

        # for each event construct publisher
        self.pubs = { 
            topic :  rospy.Publisher('~' + str(topic), BoolStamped, queue_size=1) for topic in self.topics
        }
        
    ###

    def callback(self, msg) -> None:
        
        try:
            # Reset previosu values
            for topic in self.topics:
                self.topics[topic] = False

            if msg.data<self.distance.value['eb']:
                # emergency_braking_on
                self.topics['emergency_braking'] = True

            elif     msg.data>=self.distance.value['acc']['start'] \
                and msg.data<=self.distance.value['acc']['stop']:
                # adaptive_cruise_control_on
                self.topics['adaptive_cruise_control'] = True

            elif msg.data>self.distance.value['lf']:
                # line_follower_on
                self.topics['line_follower']=True

            # Publish events
            self.msg.header.stamp= rospy.Time.now()
            for key in self.pubs:
                rospy.loginfo(f"{key} \t\t {self.topics[key]}")
                self.msg.data = self.topics[key]
                self.pubs[key].publish(self.msg)


        except Exception as ex:
            rospy.logerr(ex)
        
    def on_shutdown(self):
        self.subscriber.unregister()

if __name__ == '__main__':
    # create the node
    node = DriveController(node_name='distance_controller_node')
    rospy.spin()