#!/usr/bin/env python3
import rospy

from numpy import rad2deg as rad2deg
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

import time

from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped, Vector2D

from pid_controller import PIDController
from lqr_controller import LQRController

class WrapperController(DTROS):

    def __init__(self, node_name):
        super(WrapperController, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )

        # CHANGE THIS
        self.controler_param = DTParam('~pid_param', param_type=ParamType.DICT)
        # self.controler_param = DTParam('~lqr_param', param_type=ParamType.DICT)

        # Set maximum angular speed
        self.omega_param = DTParam('~omega_max', param_type=ParamType.FLOAT)
        self.omega_max = self.omega_param.value
        
        # Set maximum linear speed
        self.v_max_param = DTParam('~v_max', param_type=ParamType.FLOAT)
        self.v_max = self.v_max_param.value
        # Set maximum linear speed
        self.delta_t = DTParam('~delta_t', param_type=ParamType.FLOAT)
        self.adjustment_time = DTParam('~adjustment', param_type=ParamType.FLOAT)

        # Internal timer
        self.adjustment_start_time = None
        self.adjustment_done = False

        # Subscribe to the desired point
        self.real_point = rospy.Subscriber('~desired_point', Vector2D, self.callback, queue_size=1)

        # Publishers
        self.pub_control = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)

    def callback(self, msg) -> None:
        try:
            if self.adjustment_done:
                return

            # Start timing on first callback
            if self.adjustment_start_time is None:
                self.adjustment_start_time = time.time()

            elapsed = time.time() - self.adjustment_start_time
            if elapsed > self.adjustment_time.value:
                rospy.loginfo("Adjustment time completed.")
                self.adjustment_done = True
                self.publish_zero_command()
                return

            # Get current error (assume position x for simplicity)
            target_x = self.desired_point["x"]
            error = target_x - msg.x

            # Update PID controller params if needed
            self.controler_param.force_update()
            self.controller.set_param(self.controler_param.value)

            # Run controller
            control_signal, _, _ = self.controller.run(0.0, error, self.delta_t.value)

            # Clamp omega to max
            omega = max(-1.0, min(1.0, control_signal))
            self.twist.omega = omega * self.omega_max.value
            self.twist.v = self.v_max.value

            self.twist.header.stamp = rospy.Time.now()
            self.pub_control.publish(self.twist)

        except Exception as e:
            rospy.logerr("Error in callback: {0}".format(e))

    def publish_zero_command(self):
        self.twist.omega = 0.0
        self.twist.v = 0.0
        self.twist.header.stamp = rospy.Time.now()
        self.pub_control.publish(self.twist)
