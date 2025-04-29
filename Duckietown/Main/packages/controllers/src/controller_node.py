#!/usr/bin/env python3
import rospy

from numpy import rad2deg as rad2deg
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped, FSMState

from pid_controller import PIDController
from lqr_controller import LQRController

class WrapperController(DTROS):

    def __init__(self, node_name):
        super(WrapperController, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )
        # Read controller parameters

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

        # Controller variables
        self.prev_e = 0.0
        self.prev_int = 0.0

        self.sub_fsm_mode = rospy.Subscriber("fsm_node/mode",FSMState, self.cbFSMMode, queue_size=1) 

        self.v_sub = rospy.Subscriber('~v', Float32, self.v_callback, queue_size=1)


        # CHANGE THIS
        # Create controller
        self.controller = PIDController()
        # self.controller = LQRController()
        
        # Subscribe to error topic
        self.error_sub = rospy.Subscriber('~error', Float32, self.callback, queue_size=1)
    
        # Message to publish
        self.twist = Twist2DStamped()

        # Publishers
        self.control_pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)

    def P_callback(self,msg):
        self.Kp = msg.data
    
    def I_callback(self,msg):
        self.Ki = msg.data

    def D_callback(self,msg):
        self.Kd = msg.data 


    def callback(self, msg) -> None:
        # rospy.loginfo(f"Running callback with error {msg.data}")
        try:
            
            # Set controller paramters (can by changed during tune process)
            self.controler_param.force_update()
            self.controller.set_param(self.controler_param.value)

            # Compute controll
            pd_value,error,e_int = self.controller.run(0.0, msg.data, self.delta_t.value)
            pd_value = pd_value
            # Scalling output form controller
            self.twist.omega = self.omega_max * pd_value
            
            # rospy.loginfo(pd_value)
            self.twist.v = self.v_max
            # self.twist.omega = self.omega_max.value * pd_value

            self.twist.header.stamp = rospy.Time.now()
            self.control_pub.publish(self.twist)
            # rospy.loginfo(f"Should publish {self.twist.omega} {self.twist.v}")

        except Exception as e:
            rospy.logerr("Error: {0}".format(e))

    def on_switch_on(self):
        rospy.loginfo("Controller switched from off to on")

    def on_switch_off(self): # The node will shut down before this code is run, so it's effectively useless, but it reminds us of inevitable death upon which we can take no action and after which we will serve no purpose
        pass 

    def on_shutdown(self):
        # Send stop command
        self.twist.omega = 0
        self.twist.v = 0
        rospy.Rate(100)
        temp_twist = Twist2DStamped()
        for i in range(1000):
            self.twist.header.stamp = rospy.Time.now()
            self.control_pub.publish(self.twist)
            if i == 500:
                temp_twist.omega = 0.01
                temp_twist.v = 0
                temp_twist.header.stamp = rospy.Time.now()
                self.control_pub.publish(temp_twist)
        rospy.sleep(2)
        rospy.loginfo("Stop Controller")
    
    def cbFSMMode(self,  msg) -> None:
        if msg.state=='LANE_FOLLOWING':
            self.v_max = self.v_max_param.value
            self.omega_max = self.omega_param.value
        else:
            self.v_max = 0
            self.omega_max = 0
            self.twist.v = self.v_max
            self.twist.omega = self.omega_max
            self.twist.header.stamp = rospy.Time.now()
            self.control_pub.publish(self.twist)

    def v_callback(self, msg) -> None:
        # rospy.loginfo(f"Changing cruise value to {msg.data}")
        self.v_max = msg.data

###################################################################################

if __name__ == '__main__':
    some_name_node = WrapperController(node_name='controller_node')
    rospy.spin()
