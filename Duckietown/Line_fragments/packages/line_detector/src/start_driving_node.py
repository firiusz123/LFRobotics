
from duckietown.DTROS import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

import rospy

from std_msgs.msg import BoolStamped

class StartDriving(DTROS):
    def __init__(self):
        super(StartDriving, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )
        self.start_publisher = rospy.Publisher("~start_driving", BoolStamped, queue_size=1 )
        self.value = False
    
    
if __name__ == "__main__":
    startDriving = StartDriving()

    rospy.spin()