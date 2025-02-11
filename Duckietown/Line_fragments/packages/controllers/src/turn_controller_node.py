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
        self.timeout_service = rospy.Service(
            "~set_timeout", Float32, self.turning_timeout
        )
        
        # Sends a signal to FSM for the robot to start moving
        self.unlock_publisher = rospy.Publisher(
            "~start_driving", BoolStamped, queue_size=1
        )

        # Gets possible robot directions and initializes a turn to a random one 
        self.turning_service = rospy.Service(
            "~get_direction", BoolStamped, self.turn
        )
        
        self.control_pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)

        self.turn_options_proxy = rospy.ServiceProxy("~get_turn_options",Int32)

    def turning_timeout(self):
        sleep_duration = randint(1,5)
        rospy.loginfo(f"Sleeping for {sleep_duration}")
        rospy.sleep(sleep_duration)

    def turn(self, turn=None):
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
        turn_number = self.turn_options_proxy
        i = 1
        options = []
        while turn_number > 0:
            turn = turn_number % 2
            if turn != 0:
                options.append(i)
            i += 1
            turn_number //= 2
        match choice(options):
            # Case for going forward
            case 1:
                self.control_pub.omega = 0
                self.control_pub.v = self.v_max # Parameter
            # Case for turning right
            case 2:
                omega_turn = self.omega_max / 4
                d_omega = omega_turn / self.steps
                self.control_pub.v = self.v_max / 2
                for i in range(1,self.steps): # Turn steps
                    self.control_pub.omega += d_omega * i
                    
            # Case for turning left
            case 3:
                omega_turn = self.omega_max / 4
                d_omega = omega_turn / self.steps
                self.control_pub.v = self.v_max / 2
                for i in range(1,self.steps): # Turn steps
                    self.control_pub.omega -= d_omega * i
            case _:
                rospy.loginfo("Can't turn anywhere, going to line following instanly")


if __name__ == "__main__":
    timeoutNode = TimeoutNode()
    rospy.spin()