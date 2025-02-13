#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
# from std_msgs.msg import Bool
from duckietown_msgs.msg import BoolStamped
from sensor_msgs.msg import CompressedImage
from duckietown.dtros import DTROS, DTParam, NodeType, ParamType

class RedLineDetector(DTROS):
    def __init__(self, node_name):
        super(RedLineDetector, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        # Initialize parameters and publishers
        self.color = DTParam('~color', param_type=ParamType.DICT)
        self.image_param = DTParam('~image_param', param_type=ParamType.DICT)
        self.search_area = DTParam('~search_area', param_type=ParamType.DICT)
        self.line_position_tolerance = DTParam('~line_tolerance', param_type=ParamType.DICT)
        self.debug_image = None
        self.debug_out_image = None

        self.cvbridge = cv_bridge.CvBridge()
        self.color_line_mask = {k: np.array(v) for k, v in self.color.value.items()}

        self.red_line_detected_pub = rospy.Publisher('~red_line_detection', BoolStamped, queue_size=1)

        # Subscribe to the image topic
        self.sub_image = rospy.Subscriber('~image/in/compressed', CompressedImage, self.callback, queue_size=1)

        self.pub_image = rospy.Publisher('~red_line/image/out/compressed', CompressedImage, queue_size=1)

        self.cont_pub_image = rospy.Publisher('~center/image/out/compressed', CompressedImage, queue_size=1)

    # ============== Should be in a util file ==============
    def calculate_center(self,image):
        
        grayscale_red = cv2.cvtColor(self.process_image(image), cv2.COLOR_BGR2GRAY)
        
        # Threshold to create a binary image (single channel)
        _, thresh_image = cv2.threshold(grayscale_red, 0, 255, cv2.THRESH_BINARY)
        
        
        self.cont_pub_image.publish(self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([thresh_image]),axis=1), 'jpg'))

        # Find contours of the object in the thresholded image
        contours, _ = cv2.findContours(thresh_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Assuming the largest500,contour is the object
        # Get the moments of the largest contour
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
        else:
            return None
        
        # Calculate moments for the largest contour
        M = cv2.moments(largest_contour)

        # Check if the moment is valid to avoid division by zero
        if M['m00'] != 0:
            # Calculate the center of mass (centroid)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # print(f"Center of Mass (Centroid): ({cx}, {cy})")
        else:
            return None
        return (cx,cy)
    # ============== Should be in a util file ==============
    
    def check_tolerance(self,point):
        if point is None:
            return False
        
        pointXdistance = self.line_position_tolerance.value['centerX'] - point[0]
        pointYdistance = self.line_position_tolerance.value['centerY'] - point[1]
        centerPoint = (self.line_position_tolerance.value['centerX'],self.line_position_tolerance.value['centerY'])
        radius = self.line_position_tolerance.value['centerTolerance']
        
        # Append debug circles to check threshold
        cv2.circle(self.debug_image,centerPoint,radius,(0,255,0),2)
        cv2.circle(self.debug_image,(point[0],point[1]),1,(255,255,255),-1)

        return self.line_position_tolerance.value['centerTolerance']**2 >= pointXdistance**2 + pointYdistance**2

    def detected_line_using_red_result(self, red_result, threshold=0.1):
        # Convert the red result to grayscale to count non-zero intensities
        grayscaleRed = cv2.cvtColor(red_result, cv2.COLOR_BGR2GRAY)

        # Count the number of non-zero pixels
        nonZeroPixels = np.count_nonzero(grayscaleRed)

        # Calculate the total number of pixels in the image
        totalPixels = grayscaleRed.size

        # Calculate the percentage of non-zero pixels (red intensity detected)
        redPercentage = nonZeroPixels / totalPixels

        return True if redPercentage >= threshold else False

    def process_image(self, image):
        # Crop the top half of the image (retain only the bottom half)
        height, width = self.image_param.value['height'], self.image_param.value['width']
        cropped_image = image[height // 2:, :]  # Retain only the bottom half

        # Apply Gaussian Blur to smooth the image
        blurred_image = cv2.GaussianBlur(cropped_image, (21, 21), 0)

        # Convert the blurred image to HSV color space
        hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

        # Define the HSV range for detecting red
        # lower_red1 = np.array([0, 120, 70])
        # upper_red1 = np.array([10, 255, 255])
        # lower_red2 = np.array([170, 120, 70])
        # upper_red2 = np.array([180, 255, 255])

        # Create masks for the red color ranges
        lower_mask = cv2.inRange(hsv_image,self.color_line_mask['lower1'], self.color_line_mask['upper1'])
        upper_mask = cv2.inRange(hsv_image, self.color_line_mask['lower2'], self.color_line_mask['upper2'])

        # mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        # mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(lower_mask,upper_mask)

        # Apply the mask to the blurred image to extract red regions
        red_result = cv2.bitwise_and(blurred_image, blurred_image, mask=red_mask)

        self.debug_image = red_result

        return red_result
        # return self.detected_line_using_red_result(red_result)

    def line_detected(self,image):
        return self.detected_line_using_red_result(self.process_image(image))

    def callback(self, msg) -> None:
        try:
            # Convert the compressed image message to an OpenCV image
            image = self.cvbridge.compressed_imgmsg_to_cv2(msg)

            # Process the image to check if the red line is detected
            lineInThreshold = self.line_detected(image)
            # rospy.loginfo(self.calculate_center(image))
            lineInCenter = self.check_tolerance(self.calculate_center(image))

            # Create a BoolStamped message to publish the result
            # rospy.loginfo(f"Threshold {lineInThreshold} and Center {lineInCenter}")
            lineDetected = lineInThreshold and lineInCenter
            bool_msg = BoolStamped()
            bool_msg.header.stamp = rospy.Time.now()
            bool_msg.data = bool(lineDetected)


            self.debug_out_image = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([self.debug_image]),axis=1), 'jpg')
            self.debug_out_image.header.stamp = rospy.Time.now()
            self.pub_image.publish(self.debug_out_image)
            # Publish the result
            self.red_line_detected_pub.publish(bool_msg)
            rospy.loginfo(f"Line detected: {lineDetected}")

        except cv_bridge.CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

if __name__ == '__main__':
    # Create the ROS node
    red_line_detector_node = RedLineDetector(node_name='red_line_detection_node')
    
    # Keep the node running
    rospy.spin()
