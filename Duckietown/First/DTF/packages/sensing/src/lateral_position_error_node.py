#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np

# import DTROS-related classes
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

# import messages and services
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

class LateralPositionError(DTROS):

    def __init__(self, node_name):
        super(LateralPositionError, self).__init__(
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

        # Messages
        self.error = {'raw' : None, 'norm' : None}

        # Subscribe to image topic
        self.sub_image = rospy.Subscriber('~image/in/compressed', CompressedImage, self.callback, queue_size=1)
        
        # Publishers
        self.pub_error = {
            'raw'     : rospy.Publisher('~error/raw/lateral', Float32, queue_size=1),
            'norm'    : rospy.Publisher('~error/norm/lateral', Float32, queue_size=1)
        }

        # Transformed image
        self.pub_debug_img = rospy.Publisher('~debug/image/out/compressed', CompressedImage, queue_size=1)

        rospy.loginfo("Normalize factor: {0}".format(self.normalize_factor))
        rospy.loginfo("Follow line color: {0}".format( self.color.value['name'] ))

    def callback(self, msg) -> None:
        try:
            # Read input image
            # A - Place your code here
            image = self.cvbridge.compressed_imgmsg_to_cv2(msg)

            # Convert image to HSV color space
            # B - Place your code here
            image_hsv = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)

            # Find follow line
            # C - Place your code here
            #lower boundary
            lower_mask = (image_hsv,self.color_line_mask['lower1'], self.color_line_mask['upper1'])
            
            
            # upper boundary
            upper_mask = (image_hsv,self.color_line_mask['lower2'], self.color_line_mask['upper2'])
            # Mask image
            full_mask = cv2.inRange(lower_mask,upper_mask)

            result_mask = cv2.bitwise_and(image, image, mask=full_mask)

            # Cut image, only consider 75% of image area

            # D - Place your code here       
            result_mask[self.search_area.value['top']:self.search_area.value['bottom'],self.image_param.value['height']:self.image_param.value['width']]
            
            # Find center of mass detected yellow line
            # E - Place your code here
            M = cv2.moments(result_mask)
 
            # calculate x,y coordinate of center
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cx_0 = self.search_area.value['bottom'] - self.search_area.value['top']
            cy_0 = self.image_param.value['width']
            # Estimate error
            # F - Place your code here
            self.error['raw'] = cx_0 - cx
            self.error['norm'] = cy_0 - cy
            # Publish error
            # G - Place your code here
            self.pub_error['raw'].publish(self.error['raw'])
            self.pub_error['norm'].publish(self.error['norm'])

            # DEBUG
            if self.pub_debug_img.anybody_listening():
                
                # Add circle in point of center of mass
                cv2.circle(image, (int(cx), int(cy)), 10, (0,255,0), -1)
                
                # Add error value to image
                cv2.rectangle(image, (0, 0), (self.image_param.value['width'], 50), (255,255,255), -1)

                cv2.putText(image, "Error= " + str(self.error['raw']), 
                    org=(10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0,0,0), fontScale=0.5, 
                    thickness=1, lineType=cv2.LINE_AA)
                
                cv2.putText(image, "Error normalize= " + str( "%.3f" % self.error['norm']), 
                    org=(10,40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0,0,0), fontScale=0.5, 
                    thickness=1, lineType=cv2.LINE_AA)
                
                cv2.circle(result_mask, (int(cx), int(cy)), 10, (0,255,0), -1)
                cv2.line(result_mask, 
                    (0, self.search_area.value['top']), (self.image_param.value['width'], self.search_area.value['top']), 
                    (0, 255, 0), 2)
                cv2.line(result_mask, 
                    (0, self.search_area.value['bottom']), (self.image_param.value['width'], self.search_area.value['bottom']), 
                    (0, 255, 0), 2)
                
                # Message data
                debug_out_image = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([image], [result_mask]),axis=0).reshape(
                    (2*self.image_param.value['height'], self.image_param.value['width'], 3)), 'jpg')
                debug_out_image.header.stamp = rospy.Time.now()
                
                # Publish transformed image
                self.pub_debug_img.publish(debug_out_image)

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        
if __name__ == '__main__':
    some_name_node = LateralPositionError(node_name='lateral_position_error_node')
    rospy.spin()