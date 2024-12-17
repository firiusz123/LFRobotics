#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from math import sqrt
print("Lorem ipsum")

# import DTROS-related classes
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

# import messages and services
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

class LateralPositionError(DTROS):

    def __init__(self, node_name):
        super(LateralPositionError, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )

        self.points = []
        self.points1 = []
        self.points2 = []
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
        self.points_pub = { 
            'x': rospy.Publisher('~detected_points/x', Float32MultiArray, queue_size=1),
            'y': rospy.Publisher('~detected_points/y', Float32MultiArray, queue_size=1)
        }

        # Transformed image
        self.pub_debug_img = rospy.Publisher('~image/out/compressed', CompressedImage, queue_size=1)

        # rospy.loginfo("Normalize factor: {0}".format(self.normalize_factor))
        # rospy.loginfo("Follow line color: {0}".format( self.color.value['name'] ))
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
            print("Part omited")
            return None
        # Calculate moments for the largest contour
        
        M = cv2.moments(largest_contour)
        # Check if the moment is valid to avoid division by zero
        
        if M['m00'] != 0:
            # Calculate the center of mass (centroid)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            print(f"Center of Mass (Centroid): ({cx}, {cy})")
        else:
            print("Part omited")
            return None
        return (cx,cy)
    
    def callback(self, msg) -> None:
        try:           
            # Read input image
            image = self.cvbridge.compressed_imgmsg_to_cv2(msg)

            # Convert image to HSV color space
            image_hsv = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)

            # Find follow line
            lower_mask = cv2.inRange(image_hsv,self.color_line_mask['lower1'], self.color_line_mask['upper1'])
            upper_mask = cv2.inRange(image_hsv, self.color_line_mask['lower2'], self.color_line_mask['upper2'])

            # Combine the masks
            full_mask = cv2.bitwise_or(lower_mask, upper_mask)

            # Mask image
            result_mask = cv2.bitwise_and(image, image, mask=full_mask)

            # Chunk image
            chunks = self.chunk_image(result_mask)
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
            rospy.loginfo(f"Calculated centers {centers}")
            self.points = []
            counter = 0
            acc_x = 0
            acc_y = 0
            for center in centers:
                if center:
                    acc_x += center[0]
                    acc_y += center[1]
                    counter += 1
                else:
                    if counter > 0:
                        self.points.append((acc_x/counter,acc_y/counter))
                    acc_x = 0
                    acc_y = 0
                    counter = 0 
            if counter > 0:
                self.points.append((acc_x/counter,acc_y/counter))
            self.points2 = self.points1
            self.points1 = self.points
            # Publish error
            rospy.loginfo(f"Publishing data {self.points}")
            # self.pub_error['raw'].publish(self.error['raw'])
            # self.pub_error['norm'].publish(self.error['norm'])
            # DEBUG
            x, y = zip(*self.points)
            x, y = list(x),list(y)
            msg_x = Float32MultiArray()
            msg_x.data = x
            msg_y = Float32MultiArray()
            msg_y.data = y
            self.points_pub['x'].publish(msg_x)
            self.points_pub['y'].publish(msg_y)
            if self.pub_debug_img.anybody_listening():
                # Add circle in point of center of mass
                # for i,points in enumerate(self.points):
                #     cv2.circle(image, (int(points[0]), int(points[1]) + int(self.search_area.value['top'])), 10, (i*10,255,0), -1)
                
                for i,center in enumerate(self.points):
                    if center:
                        cv2.circle(image, (int(center[0]), int(center[1])), 10, ((i*20)%256,255,0), -1)
                # if self.points1:
                #     for i,center in enumerate(self.points):
                #         if center:
                #             cv2.circle(image, (int(center[0]), int(center[1])), 10, (0,255,(i*20)%256), -1)

                # if self.points2:
                #     for i,center in enumerate(self.points):
                #         if center:
                #             cv2.circle(image, (int(center[0]), int(center[1])), 10, (255,(i*20)%256,0), -1)

                # Add error value to image
                cv2.rectangle(image, (0, 0), (self.image_param.value['width'], 50), (255,255,255), -1)

                cv2.putText(image, "Points : " + str(self.points), 
                    org=(10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0,0,0), fontScale=0.5, 
                    thickness=1, lineType=cv2.LINE_AA)

                # cv2.circle(result_mask, (int(cx), int(cy) + int(self.search_area.value['top'])), 10, (0,255,0), -1)
                # cv2.circle(result_mask, (int(cx), int(cy)), 10, (0,255,0), -1)
                # for i in range(n-1):
                #     cv2.line(image, 
                #     (0, top + dh*i), (self.image_param.value['width'], top + dh*i), 
                #     (0, 255, 0), 2)
                #     cv2.line(image, 
                #     (0, top + dh * (i+1)), (self.image_param.value['width'], top + dh * (i+1)), 
                #     (0, 255, 0), 2)
                
                # Message data
                debug_out_image = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([image]),axis=1).reshape(
                    (self.image_param.value['height'], self.image_param.value['width'], 3)), 'jpg')
                debug_out_image.header.stamp = rospy.Time.now()
                
                # Publish transformed image
                self.pub_debug_img.publish(debug_out_image)

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        
if __name__ == '__main__':
    some_name_node = LateralPositionError(node_name='lateral_position_error_node')
    rospy.spin()