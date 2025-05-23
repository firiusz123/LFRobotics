#!/usr/bin/env python3
import rospy

# import DTROS-related class
from duckietown.dtros import \
    DTROS, \
    NodeType

# import messages and services
import message_filters
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped

class DataSync(DTROS):

    def __init__(self, node_name):
        
        super(DataSync, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )

        # Subscribe to input image topic
        self.sub_image  = message_filters.Subscriber('~in/image/compressed', CompressedImage)
        self.car_cmd    = message_filters.Subscriber('~in/car_cmd', Twist2DStamped)

        # Publisher
        self.pub_image = rospy.Publisher('~out/image/compressed', CompressedImage, queue_size=1)
        self.pub_car_cmd = rospy.Publisher('~out/car_cmd', Twist2DStamped, queue_size=1)

        # TimeSynchronizer
        self.sub_ts = message_filters.ApproximateTimeSynchronizer([self.image, self.car_cmd], 
            slop = 1, 
            queue_size=10)

        self.ts.registerCallback(self.callback)

    def callback(self, msg_image, msg_car_cmd) -> None:
        
        # Synchronize message image with car_cmd
        msg_car_cmd.header.stamp = msg_image.header.stamp
        
        # Publish synchronised messages
        self.pub_image.publish(msg_image)
        self.pub_car_cmd.publish(msg_car_cmd)


###################################################################################

if __name__ == '__main__':
    some_name_node = DataSync(node_name='data_sync_node')
    rospy.spin()