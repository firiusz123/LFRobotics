# ========= STANDARD ROS IMPORTS START =========  
import rospy

from duckietown.dtros import    DTROS, \
                                DTParam, \
                                NodeType, \
                                ParamType
# ========== STANDARD ROS IMPORTS END ==========  

# ============ MESSAGE TYPES START =============
from std_msgs.msg import Float32

from duckietown_msgs import BoolStamped
# ============= MESSAGE TYPES END ==============

class DistanceActivationNode(DTROS):

    def __init__(self, nodeName):
        super.__init__(
            nodeName=nodeName,
            node_type=NodeType.PERCEPTION 
            )

        self.nodeName = nodeName

        # Distance subscriber remaped from 
        self.subscriber     = rospy.Subscriber("~in", Float32, self.callback, queue_size=1)

        # Activation for the FSM
        self.publisher      = rospy.Publisher("~activate_acc", BoolStamped)

        self.accNodeState = BoolStamped()

        self.upper          = DTParam(f"~{nodeName}_upper", param_type=ParamType.FLOAT)

        self.lower          = DTParam(f"~{nodeName}_lower", param_type=ParamType.FLOAT)

    def debug(self):
        print(f"Node {self.nodeName} currently subscribed to the {self.accNodeState}")

    def callback(self, message):
        
        value = message.value
        
        if value > self.upper or value < self.lower:
            self.accNodeState.value = False
        else:
            self.accNodeState.value = True
        
        self.accNodeState.header.stamp = rospy.timer.now() 
        self.publisher()

