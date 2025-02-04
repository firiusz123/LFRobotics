#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import threading
import numpy as np
from math import sqrt

# import DTROS-related classes
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

# import messages and services
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8MultiArray,MultiArrayDimension

class Binmask(DTROS):
    def __init__(self, node_name):
        super(Binmask, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        
        
        
        
        # Read color mask  
        self.color = DTParam('~color', param_type=ParamType.DICT)

        # Camera parameters
        self.image_param = DTParam('~image_param', param_type=ParamType.DICT)

        # Search area of followed line
        self.search_area = DTParam('~search_area', param_type=ParamType.DICT)

        # Convert color mask to np.array
        self.color_line_mask = {k : np.array(v) for k, v in self.color.value.items()}
        
        # Normalization factor
        self.normalize_factor = float(1.0 / (self.image_param.value['width'] / 2.0))
        
        self.cvbridge = cv_bridge.CvBridge()
        
        

        # Publishers
        self.pub_mask = rospy.Publisher('~image/mask/compressed',CompressedImage,queue_size = 1)
        # Subscribe to image topic
        self.sub_image = rospy.Subscriber('~image/in/compressed', CompressedImage, self.callback, queue_size=1)
        
        # Publishers
        
        # Transformed image
        self.pub_debug_img = rospy.Publisher('~image/out/compressed', CompressedImage, queue_size=1)

        # rospy.loginfo("Normalize factor: {0}".format(self.normalize_factor))
        # rospy.loginfo("Follow line color: {0}".format( self.color.value['name'] ))





    
    def callback(self, msg) -> None:
        try:           
            # Read input image
            
            image = self.cvbridge.compressed_imgmsg_to_cv2(msg)
            image = cv2.blur(image,(5,5))
            # Convert image to HSV color space
            image_hsv = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)

            # Find follow line
            lower_mask = cv2.inRange(image_hsv,self.color_line_mask['lower1'], self.color_line_mask['upper1'])
            upper_mask = cv2.inRange(image_hsv, self.color_line_mask['lower2'], self.color_line_mask['upper2'])

            # Combine the masks
            full_mask = cv2.bitwise_or(lower_mask, upper_mask)

            # Mask image
            result_mask = cv2.bitwise_and(image,image, mask=full_mask)

            #Mask publishing
            msg_mask = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([result_mask]),axis=1).reshape(
                (self.image_param.value['height'], self.image_param.value['width'], 3)), 'jpg')
            msg_mask.header.stamp = rospy.Time.now()
            self.pub_mask.publish(msg_mask)
            
            #rospy.loginfo(f"mask size {mask_shape[0]}x{mask_shape[1]}")
            
            #Image publishing
            debug_out_image = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([image]),axis=1).reshape(
               (self.image_param.value['height'], self.image_param.value['width'], 3)), 'jpg')
            debug_out_image.header.stamp = rospy.Time.now()
            self.pub_debug_img.publish(debug_out_image)

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        
if __name__ == '__main__':
    # ===================== TO BE REMOVED ===================== 
    import warnings
    warnings.filterwarnings("ignore")
    # ===================== TO BE REMOVED =====================

    some_name_node = Binmask(node_name='binmask_node')
    rospy.spin()
    