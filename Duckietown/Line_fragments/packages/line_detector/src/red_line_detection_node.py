#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
# from std_msgs.msg import Bool
# from geometry_msgs import BoolStamped
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

        self.cvbridge = cv_bridge.CvBridge()
        self.color_line_mask = {k: np.array(v) for k, v in self.color.value.items()}

        # self.red_line_detected_pub = rospy.Publisher('~red_line_detection', BoolStamped, queue_size=1)

        # Subscribe to the image topic
        self.sub_image = rospy.Subscriber('~image/in/compressed', CompressedImage, self.callback, queue_size=1)

    def detected_line_using_red_result(self, red_result, threshold=0.1):
        # Convert the red result to grayscale to count non-zero intensities
        grayscale_red = cv2.cvtColor(red_result, cv2.COLOR_BGR2GRAY)

        # Count the number of non-zero pixels
        non_zero_pixels = np.count_nonzero(grayscale_red)

        # Calculate the total number of pixels in the image
        total_pixels = grayscale_red.size

        # Calculate the percentage of non-zero pixels (red intensity detected)
        red_percentage = non_zero_pixels / total_pixels

        return 1 if red_percentage >= threshold else 0

    def process_image(self, image):
        # Crop the top half of the image (retain only the bottom half)
        height, width, _ = image.shape
        cropped_image = image[height // 2:, :]  # Retain only the bottom half

        # Apply Gaussian Blur to smooth the image
        blurred_image = cv2.GaussianBlur(cropped_image, (21, 21), 0)

        # Convert the blurred image to HSV color space
        hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

        # Define the HSV range for detecting red
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for the red color ranges
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask = mask1 | mask2

        # Apply the mask to the blurred image to extract red regions
        red_result = cv2.bitwise_and(blurred_image, blurred_image, mask=red_mask)

        return self.detected_line_using_red_result(red_result)

    def callback(self, msg) -> None:
        try:
            # Convert the compressed image message to an OpenCV image
            image = self.cvbridge.compressed_imgmsg_to_cv2(msg)

            # Process the image to check if the red line is detected
            line_detected = self.process_image(image)

            # Create a BoolStamped message to publish the result
            # bool_msg = BoolStamped()
            # bool_msg.header.stamp = rospy.Time.now()
            # bool_msg.data = bool(line_detected)

            # Publish the result
            # self.red_line_detected_pub.publish(bool_msg)
            rospy.loginfo(f"Line detected: {line_detected}")

        except cv_bridge.CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

if __name__ == '__main__':
    # Create the ROS node
    red_line_detector_node = RedLineDetector(node_name='red_line_detection_node')
    
    # Keep the node running
    rospy.spin()
