#!/usr/bin/env python3

import rospy
 
from std_msgs.msg import Float32, Int32

from duckietown_msgs.msg import BoolStamped

from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

from duckietown_msgs.msg import Twist2DStamped, FSMState

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

        self.turn_values = DTParam('~turn_values', param_type=ParamType.DICT).value
        # Gets possible robot directions and initializes a turn to a random one 
        # self.turn_options_proxy = rospy.ServiceProxy("~get_turn_options", Int32)


        # Callback for change of states in FSM
        self.sub_fsm_mode = rospy.Subscriber("fsm_node/mode",FSMState, self.cbFSMMode, queue_size=1) 

        self.turn_routine = False

        self.CALLBACK_METHOD = False

        self.check_turn_routine_timer = rospy.Timer(rospy.Duration(0.01), self.check_turn_routine)


    def turning_timeout(self):
        sleep_duration = randint(5,7)
        rospy.loginfo(f"Sleeping random timeout for {sleep_duration}")
        rospy.sleep(sleep_duration)

    def turn_left(self):
        rospy.loginfo("Turning left")
        v = self.turn_values['left_turn']['v']
        radius = self.turn_values['left_turn']['radius']
        time = self.turn_values['left_turn']['time']
        scale_factor = self.turn_values['left_turn']['scale_factor']

        omega = v/radius * scale_factor
        self.turn_control.v = v
        self.turn_control.omega = omega

        self.turn_control.header.stamp = rospy.Time.now()
        for _ in range(1000):
            self.control_pub.publish(self.turn_control)
        rospy.sleep(time)

    def turn_right(self):
        rospy.loginfo("Turning right")
        v = self.turn_values['right_turn']['v']
        radius = self.turn_values['right_turn']['radius']
        time = self.turn_values['right_turn']['time']
        scale_factor = self.turn_values['right_turn']['scale_factor']

        omega = -v/radius * scale_factor # To turn right the value has to be negative
        self.turn_control.v = v
        self.turn_control.omega = omega

        self.turn_control.header.stamp = rospy.Time.now()
        for _ in range(1000):
            self.control_pub.publish(self.turn_control)
        rospy.sleep(time)

    def go_forward(self):
        rospy.loginfo("Going forward")
        v = self.turn_values['forward_speed']['v']
        omega = self.turn_values['forward_speed']['omega'] # Here we just set r = omega = 0, but actually r = INF, because it's a straight line
        time = self.turn_values['forward_speed']['time']

        self.turn_control.v = v
        self.turn_control.omega = omega

        self.turn_control.header.stamp = rospy.Time.now()
        for _ in range(1000):
            self.control_pub.publish(self.turn_control)
        rospy.sleep(time)

    def go_forward_short(self):
        rospy.loginfo("Going forward a short distance")
        v = self.turn_values['short_forward_speed']['v']
        omega = self.turn_values['short_forward_speed']['omega'] # Here we just set r = omega = 0, but actually r = INF, because it's a straight line
        time = self.turn_values['short_forward_speed']['time']

        self.turn_control.v = v
        self.turn_control.omega = omega

        self.turn_control.header.stamp = rospy.Time.now()
        for _ in range(1000):
            self.control_pub.publish(self.turn_control)
        rospy.sleep(time)

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
            elif option == 3:
                self.turn_right()
            # case _:
            else:
                self.go_forward_short()
                rospy.logerr("Unknown turn direction")
        except Exception as e:
            self.transmit_resume()
            rospy.loginfo(f"An error has occured in the tunr node:\n {e}")
        rospy.loginfo("Returning to following")
        self.signal_stop()
        rospy.sleep(self.sleep_duration)
        self.transmit_resume()

    def set_turn_option(self,msg):
        rospy.loginfo(f"* * * Set turn options to {msg.data}")
        # rospy.loginfo(f"* * * Acitve {self._switch}")
        self.turn_options = msg.data

    def turn_decision(self):
        """
        Function decides in which direction robot is going to turn
        """
        
        """
        Turn number determines in which direction robot can go, it's a number between 0 and 7, with bits x, y and z defined:
        xyz
        |||-> Going forward
        ||--> Left turn
        |---> Right turn 
        """
        rospy.loginfo("Turning my turing")
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
            self.go_forward_short()
            self.transmit_resume()
            return
        
        rospy.loginfo(f"Those are your options here: {options}") 
        chosen = choice(options)
        rospy.loginfo(f"You have been chosen (1 - f, 2 - l, 3 - r) {chosen}")
        self.turn(chosen)

    def transmit_resume(self):
        message = BoolStamped()
        message.header.stamp = rospy.Time.now()
        message.data = True
        for _ in range(1000):
            self.unlock_publisher.publish(message)
        
    def signal_stop(self):
        self.turn_control.v = 0
        self.turn_control.omega = 0
        # rospy.loginfo("STOPINGSTOPINGSTOPING")

        self.turn_control.header.stamp = rospy.Time.now()
        for _ in range(1000):
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

    def cbFSMMode(self, msg):
        if msg.state == 'STOPPED':
            if self.CALLBACK_METHOD:
                self.turn_routine = True
            else:
                self.turning_routine()

        else:
            self.turn_routine = False

    def check_turn_routine(self, event):
        # Periodically check if turn_routine should be executed
        if self.turn_routine and self.CALLBACK_METHOD:
            self.turning_routine()

if __name__ == "__main__":
    timeoutNode = TurningNode("turn_controller_node")
    rospy.spin()