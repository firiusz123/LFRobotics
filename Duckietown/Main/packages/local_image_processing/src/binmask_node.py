#!/usr/bin/env python3

import rospy # type: ignore
import cv2 # type: ignore
import cv_bridge # type: ignore
import numpy as np # type: ignore
from math import sqrt # type: ignore

# import DTROS-related classes
from duckietown.dtros import DTROS,DTParam, NodeType, ParamType # type: ignore

# import messages and services # type: ignore
from sensor_msgs.msg import CompressedImage # type: ignore
from std_msgs.msg import Int8MultiArray,MultiArrayDimension  # type: ignore

class Binmask(DTROS):

    def __init__(self, node_name):

        super(Binmask, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        
        # Read color mask  
        self.color = DTParam('~color', param_type=ParamType.DICT)

        # Camera parameters
        self.imageParam = DTParam('~image_param', param_type=ParamType.DICT)

        # Search area of followed line
        self.searchArea = DTParam('~search_area', param_type=ParamType.DICT)

        #need to get rid of this asap
        self.default_target_image = cv2.imread("/code/catkin_ws/src/Main/packages/local_image_processing/config/binmask_node/frame_0.png")
        # Convert color mask to np.array
        self.colorLineMask = {k : np.array(v) for k, v in self.color.value.items()}
        
        # Normalization factor
        self.notmalizeFactor = float(1.0 / (self.imageParam.value['width'] / 2.0))
        
        self.cvbridge = cv_bridge.CvBridge()
        
        # =========== Publishers =========== 

        # Ready mask for later processing
        self.pub_mask = rospy.Publisher('~image/mask/compressed',CompressedImage,queue_size = 1)

        # HSV image for alternative image display and debugging
        self.pubHSV = rospy.Publisher('~image/mask/hsv/compressed', CompressedImage,queue_size = 1)

        # =========== Subscribers =========== 

        # Subscribe to image topic
        self.sub_image = rospy.Subscriber('~image/in/compressed', CompressedImage, self.callback, queue_size=1)
                
        # Transformed image
        self.pubDebugImage = rospy.Publisher('~image/out/compressed', CompressedImage, queue_size=1)


    def match_histograms(self, source, target_bgr):
        # Convert BGR to LAB
        source_lab = cv2.cvtColor(source, cv2.COLOR_BGR2LAB)
        # rospy.loginfo("Source")
        target_lab = cv2.cvtColor(target_bgr, cv2.COLOR_BGR2LAB)
        # rospy.loginfo("Target")

        matched_lab = np.zeros_like(source_lab)

        # For each channel in LAB
        for ch in range(3):
            src = source_lab[:, :, ch].ravel()
            tgt = target_lab[:, :, ch].ravel()

            # For A and B channels (ch=1,2), shift values to [0,255]
           

            # Compute histograms (256 bins over the valid range)
            src_hist, bins = np.histogram(src, bins=256, range=(0,255), density=True)
            tgt_hist, _ = np.histogram(tgt, bins=256, range=(0,255), density=True)

            src_cdf = np.cumsum(src_hist)
            tgt_cdf = np.cumsum(tgt_hist)

            src_cdf = src_cdf / src_cdf[-1]
            tgt_cdf = tgt_cdf / tgt_cdf[-1]

            lookup = np.zeros(256, dtype=np.uint8)
            for i in range(256):
                diff = np.abs(tgt_cdf - src_cdf[i])
                lookup[i] = np.argmin(diff)

            # Map source pixels
            if ch == 0:
                matched_lab[:, :, ch] = lookup[source_lab[:, :, ch]]
            else:
                matched_lab[:, :, ch] = lookup[source_lab[:, :, ch]]

        # Convert back LAB to BGR
        matched_bgr = cv2.cvtColor(matched_lab, cv2.COLOR_LAB2BGR)
        return matched_bgr

    def callback(self, msg) -> None:
        try:           
            # Read input image            
            image = self.cvbridge.compressed_imgmsg_to_cv2(msg)
            #applying the histogram matching 
            image = self.match_histograms(image , self.default_target_image)
            
            #image = cv2.blur(image,(5,5))
            
            # Convert image to HSV color space
            imageHSV = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
            imageMessageHSV = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([imageHSV]),axis=1).reshape(
               (self.imageParam.value['height'], self.imageParam.value['width'], 3)), 'jpg')
            
            # Send the converted image
            imageMessageHSV.header.stamp = rospy.Time.now()
            self.pubHSV.publish(imageMessageHSV)
            
            # Find follow line
            lowerMask = cv2.inRange(imageHSV,self.colorLineMask['lower1'], self.colorLineMask['upper1'])
            upperMask = cv2.inRange(imageHSV, self.colorLineMask['lower2'], self.colorLineMask['upper2'])

            # Combine the masks
            fullMask = cv2.bitwise_or(lowerMask, upperMask)

            # Mask image
            resultMask = cv2.bitwise_and(image,image, mask=fullMask)

            # Mask publishing
            msgMask = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([resultMask]),axis=1).reshape(
                (self.imageParam.value['height'], self.imageParam.value['width'], 3)), 'jpg')
            
            # Send the binary image
            msgMask.header.stamp = rospy.Time.now()
            self.pub_mask.publish(msgMask)
                        
            # Debug image publishing
            debug_out_image = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([image]),axis=1).reshape(
               (self.imageParam.value['height'], self.imageParam.value['width'], 3)), 'jpg')
            
            # Send the debug image
            debug_out_image.header.stamp = rospy.Time.now()
            self.pubDebugImage.publish(debug_out_image)

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        
if __name__ == '__main__':
    # ===================== TO BE REMOVED ===================== .imageParam
    import warnings
    warnings.filterwarnings("ignore")
    # ===================== TO BE REMOVED =====================

    some_name_node = Binmask(node_name='binmask_node')
    rospy.spin()
    