#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int32
from duckietown_msgs.msg import BoolStamped
from duckietown.dtros import DTROS, DTParam, NodeType, ParamType
from duckietown_msgs.msg import Twist2DStamped
from rospy.service import Service

class StopNode(DTROS):
    def __init__(self, node_name="stop_controller_node"):
        super(StopNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # Default values for stop_duration are set by DTParam
        self.stop_duration = DTParam("~stop_duration", param_type=ParamType.Float32)

        self.turn_control = Twist2DStamped()

        # Subscriber to receive stop commands
        self.stop_subscriber = rospy.Subscriber("~stop_topic", BoolStamped, self.stop_callback, queue_size=1)

        # Publisher to send control messages
        self.resume_pub = rospy.Publisher("~resume_topic", BoolStamped, queue_size=1)

        # Service to enable stopping for a given duration
        self.stop_service = rospy.Service("~stop_for_duration", Float32, self.stop_service_callback)

    def stop_callback(self, msg):
        """
        Callback function triggered when a stop signal is received.
        """
        if msg.data:
            self.stop()
            self.timeout()
        else:
            self.resume()

    def stop(self):
        """
        Stop the robot by setting velocities to zero.
        """
        self.turn_control.header.stamp = rospy.Time.now()
        self.turn_control.v = 0
        self.turn_control.omega = 0
        self.resume_pub.publish(BoolStamped(data=False))  # Optionally indicate stopping

        # Publish the stop message
        self.control_pub.publish(self.turn_control)

    def timeout(self):
        """
        Stop the robot for the duration set by stop_duration.
        """
        rospy.loginfo(f"Stopping for {self.stop_duration}")
        rospy.sleep(self.stop_duration)  # Sleep for the stop duration

    def resume(self):
        """
        Resume robot operation by setting velocities back to their default values.
        """
        self.turn_control.v = 1.0  # You can set a default velocity
        self.turn_control.omega = 0.0  # You can set a default angular velocity
        
        self.turn_control = rospy.Time.now() 
        self.control_pub.publish(self.turn_control)
        rospy.loginfo("Resuming movement")

    def stop_service_callback(self, req):
        """
        Service callback to stop the robot for a given duration.
        """
        stop_time = req.data
        rospy.loginfo(f"Stopping for {stop_time} seconds via service call.")
        self.stop()
        rospy.sleep(stop_time)
        self.resume()
        return Float32(stop_time)  # Return the duration for acknowledgment


if __name__ == "__main__":
    stopNode = StopNode()
    rospy.spin()
