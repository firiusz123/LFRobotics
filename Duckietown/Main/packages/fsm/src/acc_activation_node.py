#!/usr/bin/env python3

from distance_activation_node import DistanceActivationNode 

import rospy

class AccActivationNode(DistanceActivationNode):
    def __init__(self, nodeName):
        super.__init__(nodeName)

if __name__ == "__main__":
    accActivationNode = AccActivationNode("acc_activation_node")
    rospy.spin()