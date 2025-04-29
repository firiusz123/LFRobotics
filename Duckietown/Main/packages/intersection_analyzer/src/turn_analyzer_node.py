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

from random import randint

# import messages and services
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Float32, Int32
from duckietown_msgs.msg import Twist2DStamped
import message_filters

class TurnAnalyzer(DTROS):

    def __init__(self, node_name):
        super(TurnAnalyzer, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )

        self.pub_debug_img = rospy.Publisher('~image/out/compressed', CompressedImage, queue_size=1)
        
        self.pub_turn_options = rospy.Publisher('~options', Int32, queue_size=1)
        
        self.sub_image = rospy.Subscriber('~image/in/compressed', CompressedImage, self.callback, queue_size=1)
        self.sub_test = rospy.Subscriber('~spinme', Twist2DStamped, self.test_cb,queue_size=1)

        # Read color mask  
        self.color = DTParam('~color', param_type=ParamType.DICT)

        self.cvbridge = cv_bridge.CvBridge()
        
        self.hue = None
        self.saturation = None
        self.value = None

    def forward_threshold(self, image):
        forward_search_area = self.search_area.value['forward_search_area']
        cropped_img = image[forward_search_area['top']:forward_search_area['bottom'], :]
        # rospy.loginfo("Applying forward treshold:")
        return self.apply_threshold(cropped_img,forward_search_area['threshold'])

    def left_threshold(self, image):
        # Set left search area (1/3 of image width)
        left_search_area = self.search_area.value['left_search_area']
        cropped_img = image[left_search_area['top']:left_search_area['bottom'], left_search_area['left']:left_search_area['right']]
        # rospy.loginfo("Applying left treshold:")
        return self.apply_threshold(cropped_img,left_search_area['threshold'])

    def right_threshold(self, image):
        # Set right search area (last 1/3 of image width)
        right_search_area = self.search_area.value['right_search_area']
        cropped_img = image[right_search_area['top']:right_search_area['bottom'], right_search_area['left']:right_search_area['right']]
        # rospy.loginfo("Applying right treshold:")
        return self.apply_threshold(cropped_img,right_search_area['threshold'])

    def apply_threshold(self, cropped_img, threshold):
        # Convert image to HSV color space
        image_hsv = cv2.cvtColor(cropped_img, cv2.COLOR_RGB2HSV)

        lower_mask = cv2.inRange(image_hsv, self.color_line_mask['lower1'], self.color_line_mask['upper1'])
        upper_mask = cv2.inRange(image_hsv, self.color_line_mask['lower2'], self.color_line_mask['upper2'])

        # Combine the masks
        full_mask = cv2.bitwise_or(lower_mask, upper_mask)

        # Apply mask to image
        result_image = cv2.bitwise_and(cropped_img, cropped_img, mask=full_mask)
        
        non_zero_pixels = cv2.countNonZero(full_mask)

        # non_zero_pixels = np.count_nonzero(result_image)
        # rospy.loginfo(f"{non_zero_pixels} {threshold}")
        if non_zero_pixels > threshold:
            return 1
        else:
            return 0

    def test_cb(self,msg):
        rospy.loginfo(f"Well it's turning alright {msg.data}")

    def callback(self,msg):
        try:
            # Camera parameters
            self.image_param = DTParam('~image_param', param_type=ParamType.DICT)

            # Search area of followed line
            self.search_area = DTParam('~search_area', param_type=ParamType.DICT)
            
            # Convert color mask to np.array
            self.color_line_mask = {k : np.array(v) for k, v in self.color.value.items()}
            
            image = self.cvbridge.compressed_imgmsg_to_cv2(msg)
            # debug_out_image = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([image]),axis=1).reshape(
            #    (self.image_param.value['height'], self.image_param.value['width'], 3)), 'jpg')

            image = cv2.blur(image,(5,5))
            # Convert image to HSV color space
            image_hsv = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
            
            lower_mask = cv2.inRange(image_hsv,self.color_line_mask['lower1'], self.color_line_mask['upper1'])
            upper_mask = cv2.inRange(image_hsv, self.color_line_mask['lower2'], self.color_line_mask['upper2'])
            # else:
            #     lower_mask = cv2.inRange()
            # Combine the masks
            full_mask = cv2.bitwise_or(lower_mask, upper_mask)

            result_image = cv2.bitwise_and(image,image, mask=full_mask)

            forward_search_area = self.search_area.value['forward_search_area']
            left_search_area = self.search_area.value['left_search_area']
            right_search_area = self.search_area.value['right_search_area']
            
            # Draw forward search area boundaries
            cv2.line(result_image, (0, forward_search_area['top']), (self.image_param.value['width'], forward_search_area['top']), (0, 255, 0), 2)  # Green
            cv2.line(result_image, (0, forward_search_area['bottom']), (self.image_param.value['width'], forward_search_area['bottom']), (0, 255, 0), 2)  # Green

            # Draw left search area boundaries
            cv2.line(result_image, (left_search_area['right'], left_search_area['top']), (left_search_area['right'], left_search_area['bottom']), (255, 0, 0), 2)  # Blue

            # Draw right search area boundaries
            cv2.line(result_image, (right_search_area['left'], right_search_area['top']), (right_search_area['left'], right_search_area['bottom']), (255, 0, 0), 2)  # Blue
            
            debug_out_image = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([result_image]),axis=1).reshape(
            (self.image_param.value['height'], self.image_param.value['width'], 3)), 'jpg')
            # left_search_area = {
            #     'top': 0,
            #     'bottom': self.image_param.value['height'],
            #     'left': 0,
            #     'right': self.image_param.value['width'] // 3
            # }

            debug_out_image.header.stamp = rospy.Time.now()
            self.pub_debug_img.publish(debug_out_image)
            values = [None] * 3
            functions = [ self.forward_threshold, self.left_threshold, self.right_threshold ]
            for index, function in enumerate(functions):
                values[index] = function(result_image)
            sum_ = 0
            for i in range(3):
                sum_ += values[i] * 2**i
            self.pub_turn_options.publish(sum_)
            # self.pub_turn_options.publish(randint(1,7))

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))



        
if __name__ == '__main__':
    # ===================== TO BE REMOVED ===================== 
    import warnings
    warnings.filterwarnings("ignore")
    # ===================== TO BE REMOVED =====================

    some_name_node = TurnAnalyzer(node_name='binmask_node')
    rospy.spin()
    