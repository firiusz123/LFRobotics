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
from duckietown_msgs.msg import Twist2DStamped

class PIDController:

    def __init__(self):
        self.Kp = None
        self.Ki = None
        self.Kd = None

    def run(self, v_0, theta_ref, theta_hat, prev_e, prev_int, delta_t):
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
        # A - Place your code here 

        # Tracking error

        # integral of the error
        # anti-windup - preventing the integral error from growing too much
        e_int = max(min(e_int, 1), -1)

        # derivative of the error
        

        # PID controller for omega
        
        return v_0, omega, e, e_int
    
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
        self.omega_max = DTParam('~omega_max', param_type=ParamType.FLOAT)
        
        # Set maximum linear speed
        self.v_max = DTParam('~v_max', param_type=ParamType.FLOAT)

        # Set maximum linear speed
        self.delta_t = DTParam('~delta_t', param_type=ParamType.FLOAT)

        # Controller variables
        self.prev_e = 0.0
        self.prev_int = 0.0

        # Create controller
        self.controller = PIDController()

        # Subscribe to error topic
        self.error_sub = rospy.Subscriber('~error', Float32, self.callback, queue_size=1)
        
        # Message to publish
        self.twist = Twist2DStamped()

        # Publishers
        self.control_pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)

    def callback(self, msg) -> None:
        try:
            
            # Set controller paramters (can by changed during tune process)
            self.controler_param.force_update()
            self.controller.set_param(self.controler_param.value)

            # Compute controll
            # B - Place your code here 
            
            # Scalling output form controller
            self.twist.omega = self. omega_max.value * self.twist.omega

            # C - Place your code here 
            # Add header timestamp
            
            
            # Publish controlll
            

        except Exception as e:
            rospy.logerr("Error: {0}".format(e))

    def on_shutdown(self):
        # Send stop command
        self.control_pub.publish(Twist2DStamped(omega=0.0, v=0.0))
        rospy.sleep(1)
        self.control_pub.publish(Twist2DStamped(omega=0.0, v=0.0))
        rospy.sleep(1)
        self.control_pub.publish(Twist2DStamped(omega=0.0, v=0.0))
        # Wait....
        rospy.sleep(1)
        rospy.loginfo("Stop PIDController")


###################################################################################

if __name__ == '__main__':
    some_name_node = WrapperController(node_name='pid_controller_node')
    rospy.spin()