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
from std_msgs.msg import Float32MultiArray, Float32

class Centroids(DTROS):

    def __init__(self, node_name):
        super(Centroids, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        self.centers = []
        self.points = []
        self.polyFunction = None
        self.zeroPoint = [50 , 380]
        
        
        # Read color mask  
       
        # Camera parameters
        self.image_param = DTParam('~image_param', param_type=ParamType.DICT)
        # Search area of followed line
        self.search_area = DTParam('~search_area', param_type=ParamType.DICT)
        self.centroid_method = DTParam('~centroid_search_method', param_type=ParamType.DICT).value
        self.centroidAlgo = self.centroid_method['method']
        if self.centroidAlgo == 0:
            self.centroid_lower_threshold = self.centroid_method['centroid_lower_threshold']
            self.centroid_upper_threshold = self.centroid_method['centroid_upper_threshold']
        self.cvbridge = cv_bridge.CvBridge()

        # Subscribe to mask topic
        self.sub_image = rospy.Subscriber('~image/mask/compressed', CompressedImage, self.callback, queue_size=1)
        
        # Publishers
        self.centroids_pub = rospy.Publisher('~image/centroids', Float32MultiArray, queue_size=1)
            
    def chunk_image(self,image):
        chunks = []
        top = self.search_area.value['top']
        bottom = self.search_area.value['bottom']
        n = self.search_area.value['n']
        dh = (bottom-top)//n
        for i in range(n-1):
            chunks.append(image[top+i*dh:top+(i+1)*dh,0:self.image_param.value['width']])
        return chunks
    
    def calculate_centers(self,image):
        
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Threshold to create a binary image (single channel)
        _, thresh_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
        
        # Find contours of the object in the thresholded image
        contours, _ = cv2.findContours(thresh_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Assuming the largest contour is the object
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
            self.error_buffer.pop(0)
            self.error_buffer.append(self.error['norm'])
            cy = int(M['m01'] / M['m00'])
        else:
            return None
        return (cx,cy)
    def get_centroids_of_image(self,image):
        # Read input image
        img = self.cvbridge.compressed_imgmsg_to_cv2(image)
        # Convert image to HSV color space
        k = self.search_area.value['width_search']
        #TODO change to params from config later on 
        cropped_image = img[: , 0:k]
        
        gray_image = cv2.cvtColor(cropped_image, cv2.COLOR_HSV2BGR)
        gray_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

        # Threshold to create a binary image (single channel)
        #_, thresh_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
        # Apply binary threshold
        _, thresh1 = cv2.threshold(gray_image, 10, 255, cv2.THRESH_BINARY)
        # Find contours
        contours, _ = cv2.findContours(
            image=thresh1, 
            mode=cv2.RETR_EXTERNAL,  # Retrieves only the outer contours
            method=cv2.CHAIN_APPROX_SIMPLE  # Stores all contour points
        )
        self.points = []
        for contour in contours:
            M = cv2.moments(contour)
            A = cv2.contourArea(contour)
            rospy.loginfo(f"contour area info{A}")
            if self.centroidAlgo == 0:
                if A > self.centroid_upper_threshold or A < self.centroid_lower_threshold:
                    continue
            # Check if the moment is valid to avoid division by zero
            if M['m00'] > 10:
                # Calculate the center of mass (centroid)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.points.append((cx,cy))
                    
            

    
    def callback(self, msg) -> None:
        if self.centroidAlgo == 1:
            try:           
                # Read input image
                image = self.cvbridge.compressed_imgmsg_to_cv2(msg)
                # Convert image to HSV color space
                

                # Chunk image
                chunks = self.chunk_image(image)
                top = self.search_area.value['top']
                bottom = self.search_area.value['bottom']
                n = self.search_area.value['n']
                dh = (bottom-top)//n

                # Calculate the center points of each chunk
                centers = []
                for i,chunk in enumerate(chunks):
                    center_point = self.calculate_centers(chunk)
                    if center_point:
                        centers.append((center_point[0],center_point[1]+dh*i+top))
                    else:
                        centers.append(None)
                self.points = []
                # Calculate centers of line strips
                counter = 0
                acc_x = 0
                acc_y = 0
                for center in centers:
                    if center:
                        acc_x += center[0]
                        acc_y += center[1]
                        counter += 1
                    else:
                #publish the centers 
                        if counter > 0:
                            self.points.append((acc_x/counter,acc_y/counter))
                        acc_x = 0
                        acc_y = 0
                        counter = 0 
                if counter > 0:
                    self.points.append((acc_x/counter,acc_y/counter))
                self.centers = centers
                self.angleThreshold = 3

                # Publish the centers 
                self.send_points = Float32MultiArray()
                self.send_points.data = [coordinate for point in self.points for coordinate in point]
                self.centroids_pub.publish(self.send_points)
                # if self.pub_debug_img.anybody_listening()
            
            except cv_bridge.CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
    #############################################################################################
        #2nd type 
        elif self.centroidAlgo == 0:
            try:           
                self.get_centroids_of_image(msg)
                    
                # Publish the centers 
                if self.points:
                    self.send_points = Float32MultiArray()
                    self.send_points.data = [coordinate for point in self.points for coordinate in point]
                    self.centroids_pub.publish(self.send_points)
                    # if self.pub_debug_img.anybody_listening()
            
            except cv_bridge.CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

        
if __name__ == '__main__':
    # ===================== TO BE REMOVED ===================== 
    import warnings
    warnings.filterwarnings("ignore")
    # ===================== TO BE REMOVED =====================

    some_name_node = Centroids(node_name='dashed_line_detection_node')
    rospy.spin()