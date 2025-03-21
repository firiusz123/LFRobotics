#!/usr/bin/env python3

import rospy
import numpy as np

# import DTROS-related classes
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

# import messages and services
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import Float32

class WheelSpeed(DTROS):

    def __init__(self, node_name):
        super(WheelSpeed, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )

        self.data_prev = None
        self.time_prev = None
        self.speed_prev = Float32(0)
        self.alpha = 0.25

        self._wheel = rospy.get_param('~wheel')

        # Messages
        self.speed = {
            'raw' : Float32(), 
            'filtered' : Float32()}

        # Subscribe to image topic
        self.sub = rospy.Subscriber('~tick', WheelEncoderStamped, self.cb_speed, queue_size=1)
        
        # Publishers
        # https://sound.eti.pg.gda.pl/~greg/dsp/07-SpecjalneFiltry.html
        self.pub = { 
            'raw' : rospy.Publisher('~speed/raw', Float32, queue_size=1),
            'filtered' : rospy.Publisher('~speed/filtered', Float32, queue_size=1)}

        self.loginfo(f"Wheel {self._wheel}")

        
    def cb_speed(self, msg) -> None:
        if self.data_prev==None or self.time_prev==None:
            self.data_prev = msg.data
            self.time_prev = rospy.Time.now().to_time()
            return None
        try:
            time = rospy.Time.now().to_time()
            diff = time-self.time_prev
            
            # Raw value of speed
            self.speed['raw'].data = (msg.data-self.data_prev) / diff
            self.pub['raw'].publish(self.speed['raw'])

            # Filtered value of speed y(n) = x(n) * alpha(y(n-1))
            # self.speed['filtered'].data = self.speed['raw'].data + self.alpha * self.speed_prev.data
            self.speed['filtered'].data = (self.speed_prev.data+self.speed['raw'].data) / 2.0
            self.pub['filtered'].publish(self.speed['filtered'])

            self.data_prev = msg.data
            self.time_prev = time
            self.speed_prev = self.speed['filtered']

            #self.loginfo(f"Speed raw wheel {self._wheel}: {self.speed['raw'].data}\t filterd:{self.speed['filtered'].data}")
            #self.loginfo(f"Time diff: {diff}\tfrequency: {1/diff} Hz")
            

        except Exception as e:
            rospy.logerr(e)

        
if __name__ == '__main__':
    some_name_node = WheelSpeed(node_name="wheel_speed_node")
    rospy.spin()