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

class TurningNode(DTROS):
    
    def __init__(self, node_name):
        super(TurningNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
    
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
        
        self.sleep_duration = 2

        self.turn_options = 1

        self.turn_options_subscriber = rospy.Subscriber('~turn_options', Int32, self.set_turn_option, queue_size=1)

        self.v_max = DTParam("~turn_parameters", param_type=ParamType.DICT).value['v_max']
        self.omega_max = DTParam("~turn_parameters", param_type=ParamType.DICT).value['omega_max']
        self.steps = DTParam("~turn_parameters", param_type=ParamType.DICT).value['steps']
        # Gets possible robot directions and initializes a turn to a random one 
        # self.turn_options_proxy = rospy.ServiceProxy("~get_turn_options", Int32)

        self.check_turn_routine_timer = rospy.Timer(rospy.Duration(0.01), self.check_turn_routine)

        self.turn_routine = False

    def turning_timeout(self):
        sleep_duration = randint(5,7)
        rospy.loginfo(f"Sleeping random timeout for {sleep_duration}")
        rospy.sleep(sleep_duration)

    def turn_left(self):
        rospy.loginfo("Turning left")
        omega_turn = self.omega_max / 6
        fixed_sleep_duration = self.sleep_duration / self.steps
        d_omega = omega_turn / self.steps
        self.turn_control.v = self.v_max / 2
        turn_steps = self.steps / 1.5
        turn_steps = int(turn_steps)
        for i in range(1,turn_steps): # Turn steps
         # Turn steps
            self.turn_control.omega += d_omega * i
            self.control_pub.publish(self.turn_control)
            rospy.sleep(fixed_sleep_duration)
            rospy.loginfo(f"Sleeping for {fixed_sleep_duration}")
    
    def turn_right(self):
        rospy.loginfo("Turning right")
        omega_turn = self.omega_max / 6
        d_omega = omega_turn / self.steps
        fixed_sleep_duration = self.sleep_duration / self.steps
        self.turn_control.v = self.v_max / 4
        turn_steps = self.steps / 1.5
        turn_steps = int(turn_steps)
        for i in range(1,turn_steps): # Turn steps
            self.turn_control.omega -= d_omega * i
            self.control_pub.publish(self.turn_control)
            rospy.sleep(fixed_sleep_duration)
            rospy.loginfo(f"Sleeping for {fixed_sleep_duration}")

    def go_forward(self):
        rospy.loginfo("Going forward")
        self.turn_control.omega = 0
        self.turn_control.v = self.v_max
        self.control_pub.publish(self.turn_control)
        rospy.sleep(self.sleep_duration)
        rospy.loginfo(f"Sleeping for {self.sleep_duration}")

    def turn(self,option):
        rospy.Rate(100)
        try:
        # match option:
            # Case for going forward
            # case 1:
            if option == 1:
                self.go_forward()
            # Case for turning right
            # case 2:
            elif option == 2:
                self.turn_left()
            # Case for turning left
            # case 3:
            if option == 3:
                self.turn_right()
            # case _:
            else:
                rospy.logerr("Unknown turn direction")
        except Exception as e:
            self.transmit_resume()
            rospy.loginfo(f"An error has occured in the tunr node:\n {e}")
        rospy.loginfo("Returning to following")
        self.signal_stop()
        rospy.sleep(self.sleep_duration)
        self.transmit_resume()

    def set_turn_option(self,msg):
        rospy.loginfo(f"Set turn options to {self.turn_options}")
        self.turn_options = msg.data

    def turn_decision(self):
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
        turn_number = self.turn_options
        i = 1
        options = []
        while turn_number > 0:
            turn = turn_number % 2
            if turn != 0:
                options.append(i)
            i += 1
            turn_number //= 2
        if len(options) == 0 or options is None:
            self.transmit_resume()
            return 
        self.turn(choice(options))

    def transmit_resume(self):
        message = BoolStamped()
        message.header.stamp = rospy.Time.now()
        message.data = True
        for i in range(10):
            self.unlock_publisher.publish(message)
        
    def signal_stop(self):
        self.turn_control.v = 0
        self.turn_control.omega = 0
        # rospy.loginfo("STOPINGSTOPINGSTOPING")
        rospy.Rate(10)
        self.control_pub.publish(self.turn_control)

    
    def turning_routine(self):
        self.signal_stop()
        self.turning_timeout()
        self.turn_decision()
        self.transmit_resume()

    def on_switch_on(self):
        rospy.loginfo("Starting up turning node")
        self.turn_routine = True

    def on_switch_off(self):
        rospy.loginfo("Shutting down turning node")
        self.turn_routine = False

    def check_turn_routine(self, event):
        # Periodically check if turn_routine should be executed
        if self.turn_routine:
            self.turning_routine()

if __name__ == "__main__":
    timeoutNode = TurningNode("turn_controller_node")
    rospy.spin()