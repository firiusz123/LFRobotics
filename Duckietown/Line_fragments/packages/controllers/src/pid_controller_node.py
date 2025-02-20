#!/usr/bin/env python3
import rospy

# import DTROS-related classes
from numpy import rad2deg as rad2deg
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

# import messages and services
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped, FSMState

class PIDController:

    def __init__(self):
        self.Kp = None
        self.Ki = None
        self.Kd = None
        self.prev_e = 0.0
        self.prev_int = 0.0
        # #apparently in the code when the PIDcontroll starts there is a callback with error but we cannot pass it to the run function         

    # def run(self, error, v_0, theta_ref, theta_hat, prev_e, prev_int, delta_t):
    def run(self, theta_ref, theta_hat, delta_t):
        """
        Args:
            v_0 (:double:) linear Duckiebot speed (given).
            theta_ref (:double:) reference heading pose
            theta_hat (:double:) the current estiamted theta.
            prev_e (:double:) tracking error at previous iteration.
            prev_int (:double:) previous integral error term.
            delta_t (:double:) time interval since last call.
        returns:
            v_0 (:double:) linear velocity of the Duckiebot 
            omega (:double:) angular velocity of the Duckiebot
            e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
            e_int (:double:) current integral error (automatically becomes prev_int_y at next iteration).
        """
        # Tracking error
        error = -(theta_ref-theta_hat)
        P = self.Kp * error

        # Integral of the error can unexpectedy jump due to polynomial detection
        # Error can 
        if abs(error) < 5:
            I = self.Ki*(self.prev_int + error * delta_t)
            self.prev_int = I
        else:
            I = self.prev_int
        max_int_value = 2
        # anti-windup - preventing the integral error from growing too much
        self.prev_int = min(max(self.prev_int,-max_int_value),max_int_value)
        e_int = max(min(I, 1), -1)

        # derivative of the error
        D = self.Kd*((error - self.prev_e) / delta_t)
        self.prev_e = error
        # PID controller for omega
        omega = P + I + D        
        
        return omega, error, e_int
    
    def set_param(self, params)->None :
        self.Kp = params['Kp']
        self.Ki = params['Ki']
        self.Kd = params['Kd']
    



###################################################################################

class WrapperController(DTROS):

    def __init__(self, node_name):
        super(WrapperController, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )
        # Read controller parameters
        self.controler_param = DTParam('~pid_param', param_type=ParamType.DICT)

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


        # Create controller
        self.controller = PIDController()

        # Subscribe to error topic
        self.error_sub = rospy.Subscriber('~error', Float32, self.callback, queue_size=1)
    
        # Message to publish
        self.twist = Twist2DStamped()

        # Publishers
        self.control_pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
    
        self.publisher_2 = rospy.Publisher('~pid_controller_node', Twist2DStamped, queue_size=1)
    


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
            
            # Scalling output form controller
            self.twist.omega = self.omega_max * pd_value
            
            rospy.loginfo(pd_value)
            self.twist.v = self.v_max
            # self.twist.omega = self.omega_max.value * pd_value

            
            # Publish control
            # rospy.loginfo(f"PID Publishing {self.twist.omega} {self.twist.v}")
            self.control_pub.publish(self.twist)
            self.publisher_2.publish(self.twist)
            rospy.loginfo(f"Should publish {self.twist.omega} {self.twist.v}")

        except Exception as e:
            rospy.logerr("Error: {0}".format(e))

    def on_switch_on(self):
        rospy.loginfo("PID controller switched from off to on")

    def on_switch_off(self): # The node will shut down before this code is run, so it's effectively useless, but it reminds us of inevitable death upon which we can take no action and after which we will serve no purpose
        pass 

    def on_shutdown(self):
        # Send stop command
        self.twist.header.stamp = rospy.Time.now()
        self.twist.omega = 0
        self.twist.v = 0
        rospy.Rate(100)
        for _ in range(1000):
            self.control_pub.publish(self.twist)
        rospy.sleep(2)
        rospy.loginfo("Stop PIDController")
    
    def cbFSMMode(self,  msg) -> None:
        if msg.state=='LANE_FOLLOWING':
            self.v_max = self.v_max_param.value
            self.omega_max = self.omega_param.value
        elif msg.state=='STOPPED':
            self.v_max = 0
            self.omega_max = 0

    def v_callback(self, msg) -> None:
        rospy.loginfo(f"Changing cruise value to {msg.data}")
        self.v_max = msg.data

###################################################################################

if __name__ == '__main__':
    some_name_node = WrapperController(node_name='pid_controller_node')
    rospy.spin()
