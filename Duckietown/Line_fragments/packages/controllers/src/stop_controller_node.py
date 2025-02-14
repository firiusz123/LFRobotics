#!/usr/bin/env python3

import rospy 

from std_msgs.msg import Float32, Int32

from duckietown_msgs.msg import BoolStamped

from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

from duckietown_msgs.msg import Twist2DStamped


class StopNode(DTROS):
    def __init__(self,node_name="stop_controller_node"):
        super(StopNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self.stop_duration = DTParam("~stop_duration", param_type=ParamType.Float32)

        self.turn_control = Twist2DStamped()
        
        self.stop_subscriber = rospy.Subscriber("~stop_topic", self.stop, queue_size=1)

        self.resume = rospy.Publisher()

    def callback(self):
        self.stop()
        self.timeout()
        self.exit()
        
    def exit(self):
        pass
    def stop(self):
        self.turn_control.header.stamp = rospy.Time.now()
        self.turn_control.v = 0
        self.turn_control.omega = 0


    def timeout(self):
        rospy.loginfo(f"Stopping for {self.stop_duration}")
        rospy.sleep(self.stop_duration)

if __name__ == "__main__":
    stopNode = StopNode()
    rospy.spin()