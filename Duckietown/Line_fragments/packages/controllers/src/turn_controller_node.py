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

from random import randint, choice

class TimeoutNode(DTROS):
    def __init__(self, node_name):
        super(TimeoutNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
    
        # Service that will wait a couple of seconds before moving the robot
        # self.timeout_service = rospy.Service(
        #     "~set_timeout", Float32, self.turning_timeout
        # )
        
        # Sends a signal to FSM for the robot to start moving
        self.unlock_publisher = rospy.Publisher(
            "~start_driving", BoolStamped, queue_size=1
        )

        # self.turning_service = rospy.Service(
        #     "~turn_in_diretcion", BoolStamped, self.turn
        # )
        
        self.control_pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)

        self.turn_control = Twist2DStamped()
        
        self.v_max = DTParam("~turn_parameters", param_type=ParamType.DICT).value['v_max']
        self.omega_max = DTParam("~turn_parameters", param_type=ParamType.DICT).value['omega_max']
        self.steps = DTParam("~turn_parameters", param_type=ParamType.DICT).value['steps']
        # Gets possible robot directions and initializes a turn to a random one 
        # self.turn_options_proxy = rospy.ServiceProxy("~get_turn_options", Int32)

    def turning_timeout(self):
        sleep_duration = randint(1,5)
        rospy.loginfo(f"Sleeping for {sleep_duration}")
        rospy.sleep(sleep_duration)

    def turn(self,option):
        try:
        # match option:
            # Case for going forward
            # case 1:
            if option == 1:
                self.turn_control.omega = 0
                self.turn_control.v = self.v_max # Parameter
                self.control_pub.publish(self.turn_control)
            # Case for turning right
            # case 2:
            elif option == 2:
                omega_turn = self.omega_max / 4
                d_omega = omega_turn / self.steps
                self.turn_control.v = self.v_max / 2
                for i in range(1,self.steps): # Turn steps
                    self.turn_control.omega += d_omega * i
                    self.control_pub.publish(self.turn_control)
            # Case for turning left
            # case 3:
            if option == 3:
                omega_turn = self.omega_max / 4
                d_omega = omega_turn / self.steps
                self.turn_control.v = self.v_max / 2
                for i in range(1,self.steps): # Turn steps
                    self.turn_control.omega -= d_omega * i
                    self.control_pub.publish(self.turn_control)
            # case _:
            else:
                rospy.logerr("Unknown turn direction")
        except Exception as e:
            rospy.loginfo(f"An error has occured in the tunr node:\n {e}")

    def callback(self):
        """
        Function decides in which direction robot is going to turn
        """
        
        """
        Turn number determines in which direction robot can go, it's a number between 0 and 7, with bits x, y and z defined:
        xyz
        |||-> Going forward
        ||--> Right turn
        |---> Left turn 
        """
        # turn_number = self.turn_options_proxy
        turn_number = randint(1,4)
        i = 1
        options = []
        while turn_number > 0:
            turn = turn_number % 2
            if turn != 0:
                options.append(i)
            i += 1
            turn_number //= 2
        self.turn(choice(options))

    def transmit_resume(self):
        message = BoolStamped()
        message.header.stamp = rospy.Time.now()
        message.data = True
        for i in range(10):
            self.unlock_publisher.publish(message)
        
    def signal_stop(self):
        self.turn_control.v = 0
        rospy.loginfo("SIGNAL STOPSIGNAL STOPSIGNAL STOPSIGNAL STOPSIGNAL STOPSIGNAL STOPSIGNAL STOPSIGNAL STOPSIGNAL STOPSIGNAL STOPSIGNAL STOPSIGNAL STOPSIGNAL STOPSIGNAL STOP")
        self.turn_control.omega = 0
        self.control_pub.publish(self.turn_control)

    def on_switch_on(self):
        self.signal_stop()
        self.turning_timeout()
        # self.callback()
        self.turn_control.omega = 0
        self.turn_control.v = self.v_max # Parameter
        self.control_pub.publish(self.turn_control)
        # Will be set as a parameter later
        sleep_duration = 2
        rospy.sleep(sleep_duration)
        rospy.loginfo(f"Sleeping for {sleep_duration}")
        self.signal_stop()
        # rospy.Timer(rospy.Duration(sleep_duration),self.signal_stop,oneshot=True)
        self.transmit_resume()

if __name__ == "__main__":
    timeoutNode = TimeoutNode("turn_controller_node")
    rospy.spin()