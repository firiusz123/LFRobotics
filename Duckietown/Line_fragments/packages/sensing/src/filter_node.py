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
from sensor_msgs.msg import Range

class Filter(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(Filter, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        # Setting up parameters
        self.queue = DTParam(
            '~queue',
            param_type=ParamType.INT,
            min_value=3
        )

        self.alpha = DTParam('~alpha', ParamType.FLOAT)
        self.avg_range = Float32()

        # Min, max range sensor
        self.range = DTParam('~range', param_type=ParamType.DICT)

        self.data = np.array([])

        self.prev_range = None

        # subsriber data
        self.subscriber = rospy.Subscriber('~in', Range, self.callback, queue_size=1)

        # construct publisher
        self.pub = {
            'median'    : rospy.Publisher('~median', Float32, queue_size=1),
            'avarage'   : rospy.Publisher('~average', Float32, queue_size=1)
        }

    def callback(self, msg) -> None:
        
        # Set average filter alpha paramter (can by changed during tune process)
        self.alpha.force_update()
        
        try:
            # Check if datas are in range
            if msg.range<self.range.value['min'] : 
                msg.range = self.range.value['min']
            if msg.range>self.range.value['max'] : 
                msg.range = self.range.value['max']

            # Check if read data are in allowed range
            if      msg.range>=self.range.value['min'] \
                and msg.range<=self.range.value['max']:
                
                # Median filter
                # If queue is not full then fill it 
                if len(self.data)<self.queue.value :
                    self.data = np.append(self.data, msg.range)
                else :
                    # Move data one postion in left
                    self.data = np.roll(self.data, -1)
                    # Append last range value
                    self.data[-1] = msg.range
                    # Compute median and publish
                    self.pub['median'].publish(Float32( np.median(self.data) ))

            # Avarage filter
            if self.prev_range==None:
                self.prev_range=msg.range
                return
                
            self.avg_range.data = self.alpha.value * msg.range + (1-self.alpha.value)*self.prev_range
            self.pub['avarage'].publish(self.avg_range)
            self.prev_range = self.avg_range.data
            

        except Exception as ex:
            rospy.logerr(ex)


if __name__ == '__main__':
    # create the node
    node = Filter(node_name='filter_node')
    rospy.spin()