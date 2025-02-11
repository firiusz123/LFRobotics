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

class ACCControler(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ACCControler, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        
        # Computer v
        self.v_acc = Float32()

        # Setting up parameters
        self.range = DTParam('~range', param_type=ParamType.DICT)
        self.v = DTParam('~v', param_type=ParamType.DICT)

        # subsriber data
        self.subscriber = rospy.Subscriber('~in', Float32, self.callback, queue_size=1)

        # Publish v
        self.v_pub = rospy.Publisher('~v', Float32, queue_size=1)


    def callback(self, msg) -> None:
        try:
            
            self.v_acc.data = (self.v.value['max']-self.v.value['min'])/(self.range.value['max']-self.range.value['min']) \
                * (msg.data-self.range.value['min'])

            if self.v_acc.data<0.0 :
                self.v_acc.data=0.0
            elif self.v_acc.data>self.v.value['max'] :
                self.v_acc.data=self.v.value['max']

            self.v_pub.publish(self.v_acc)
            rospy.loginfo(self.v_acc.data)

        except Exception as ex:
            rospy.logerr(ex)


    def on_switch_on(self):
        rospy.loginfo("Acc controller switched from off to on")

    def on_switch_off(self): # The node will shut down before this code is run, so it's effectively useless, but it reminds us of inevitable death upon which we can take no action and after which we will serve no purpose
        pass 

    def on_shutdown(self):
        self.subscriber.unregister()

if __name__ == '__main__':
    # create the node
    node = ACCControler(node_name='acc_controller_node')
    rospy.spin()