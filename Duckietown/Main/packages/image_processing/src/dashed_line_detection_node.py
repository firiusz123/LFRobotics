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

class DashedLineDetector(DTROS):

    def __init__(self, node_name):
        super(DashedLineDetector, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        self.centers = []
        self.points = []
        self.points1 = []
        self.points2 = []
        self.polyFunction = None
        self.zeroPoint = [50 , 380]
        self.max = 300
        self.min = -300
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
        
        self.error = {'raw' : None, 'norm' : None}

        # Publishers
        self.pub_error = {
            'raw'     : rospy.Publisher('~error/raw/lateral', Float32, queue_size=1),
            'norm'    : rospy.Publisher('~error/norm/lateral', Float32, queue_size=1)
        }
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

    def polyfit(self):
        def notNone(object):
            return object is not None
        
        # as deafault we can assume degree of 3 
        poly_degree = 3
        threshold_triggered = 0

        filteredCenters = filter(notNone, self.points)
        x_points, y_points = [], []
        centers = []
        for element in filteredCenters:
            centers.append(element)
        if len(centers) and len(centers) >2 :
            x_points, y_points = zip(*centers)
            
            for i in range(len(centers) - 1): 
                v1 = (x_points[i] , y_points[i])
                v2 = (x_points[1+1] - x_points[i]   , y_points[i+1] - y_points[i])
                dot = v1[0] * v2[0] + v1[1] * v2[1]
                mag1 = np.sqrt(v1[0]**2 + v1[1]**2)
                mag2 = np.sqrt(v2[0]**2 + v2[1]**2)
                if mag1 == 0 or mag2 == 0:
                    return 0  
                angle_rad = np.arccos(dot / (mag1 * mag2))
                angle_deg = np.degrees(angle_rad)
                if angle_deg <= 3:
                    threshold_triggered = 1
                else:
                    threshold_triggered = 0

        if not threshold_triggered:
            poly_degree = 3 
        else:
            poly_degree = 1
        # Create an event to
        #for center in centers:  handle the timeout
        timeout_event = threading.Event()

        def fit_poly():
            try:
                np.seterr(all='ignore')  # Ignore warnings
                poly = np.polyfit(x_points, y_points, poly_degree)
                self.polyFunction = np.poly1d(poly)
                timeout_event.set()  # Mark the event as finished
            except Exception as e:
                rospy.loginfo(e)
                pass

        # Start the thread to run polyfit
        polyfit_thread = threading.Thread(target=fit_poly)
        polyfit_thread.start()

        # Wait for the thread to complete or timeout after 0.3 seconds
        if not timeout_event.wait(timeout=0.05):
            rospy.loginfo("Polyfit computation timed out, leaving polyFunction unchanged")
            # Optionally, you can also terminate the thread if needed
            # However, Python does not have a clean way to kill threads directly, so it is better to let them finish naturally
        else:
            # rospy.loginfo("Polyfit computation completed successfully.")
            polyfit_thread.join()  


    def select_closest_point_reference(self,points,referencePoint):
        def distance(a,b):
            return sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
        if len(points) == 0:
            return None
        for starting_point in points:
            if starting_point:
                result = starting_point
                break
        min_distance = distance(result,referencePoint)
        for point in points:
            if point:    
                point_distance = distance(point,referencePoint)
                if min_distance > point_distance:
                    min_distance = point_distance
                    result = point
        return result


    def select_closest_point_y(self,points,desired_y):
        def y_distance(point, y):
            return abs(point[1] - y)
        if len(points) == 0:
            return None
        for starting_point in points:
            if starting_point:
                result = starting_point
                break
        min_distance = y_distance(result,desired_y)
        for point in points:
            if point:
                point_y_distance = y_distance(point,desired_y)
                if min_distance > point_y_distance:
                    min_distance = point_y_distance
                    result = point
        return result


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
            # print("Part omited")
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
            # print("Part omited")
            return None
        return (cx,cy)
    
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
                    if counter > 0:
                        self.points.append((acc_x/counter,acc_y/counter))
                    acc_x = 0
                    acc_y = 0
                    counter = 0 
            if counter > 0:
                self.points.append((acc_x/counter,acc_y/counter))
            self.centers = centers
            self.points2 = self.points1
            self.points1 = self.points
            self.angleThreshold = 1

            
            # DEBUG
            x, y = [], []
            if self.points:
                x, y = zip(*self.points)
            x, y = list(x),list(y)
            self.polyfit()
            msg_x = Float32MultiArray()
            msg_x.data = x
            msg_y = Float32MultiArray()
            msg_y.data = y
            self.points_pub['x'].publish(msg_x)
            self.points_pub['y'].publish(msg_y)
            # if self.pub_debug_img.anybody_listening():

            for i,center in enumerate(self.points):
                if center:
                    cv2.circle(image, (int(center[0]), int(center[1])), 10, ((i*20)%256,255,0), -1)
            #draw_x = np.linspace(min(x) , max(x) , int(max(x)-min(x)))
            draw_x = np.linspace(0 , 640, 640)
            draw_y = self.polyFunction(draw_x)
            
            
            # Calculate intersection points
            intersectionXValues = (self.polyFunction - self.zeroPoint[1]).roots.real
            intersectionPoints = [ (x,self.zeroPoint[1]) for x in intersectionXValues ]
            # Get all of the green point from line fragments to 
            closestLinePoint = self.select_closest_point_y(self.points,self.zeroPoint[1])
            # Get the closest point from intersection points to the one calculated previously
            extendedLinePoint = self.select_closest_point_reference(intersectionPoints,closestLinePoint)
            
            #for intersection in intersectionXValues:
            #    cv2.circle(image, (int(intersection), self.zeroPoint[1]), 10, (0,0,255), -1)
            cv2.circle(image, (self.zeroPoint[0], int(self.zeroPoint[1])), 10, (255,0,255), -1)
        
            cv2.circle(image, (int(extendedLinePoint[0]), int(extendedLinePoint[1])), 10, (255,255,255), -1)
            
            cv2.circle(image, (int(closestLinePoint[0]), int(closestLinePoint[1])), 15, (255,0,0), -1)
                        
            candidateError = float('inf')
            # for intersectionX in intersectionXValues:
            #     candidateError = min(candidateError,(self.zeroPoint[0] - max(intersectionXValues))).real
            candidateError = self.zeroPoint[0] - closestLinePoint[0]
            self.error['raw'] = candidateError
            if self.error['raw'] > self.max:
                self.max = self.error['raw']
            if self.error['raw'] < self.min:
                self.min = self.error['raw']
            if self.max != self.min:
                norm = 2 * (self.error['raw'] - self.min) / (self.max - self.min) - 1
            else:
                norm = 0  # Or any default normalized value
            self.error['norm'] = norm.real
            
            # Publish error
            # G - Place your code here
            rospy.loginfo(f"Publishing data {self.error['raw']} {self.error['norm']}")
            self.pub_error['raw'].publish(self.error['raw'])
            self.pub_error['norm'].publish(self.error['norm'])
        
            draw_points = (np.asarray([draw_x, draw_y]).T).astype(np.int32) 
            #cv2.circle(image, (int(self.xpoint), int(self.polyFunction(self.xpoint))), 10, (255,0,0), -1)
            cv2.polylines(image, [draw_points], False,color=(255,0,0) , thickness= 2) 
            # Add error value to image
            cv2.rectangle(image, (0, 0), (self.image_param.value['width'], 50), (255,255,255), -1)
            cv2.putText(image, "Points : " + str(self.error['norm']), 
                org=(10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0,0,0), fontScale=0.5, 
                thickness=1, lineType=cv2.LINE_AA)
            debug_out_image = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([image]),axis=1).reshape(
                (self.image_param.value['height'], self.image_param.value['width'], 3)), 'jpg')
            debug_out_image.header.stamp = rospy.Time.now()
            
            # Publish transformed image
            self.pub_debug_img.publish(debug_out_image)

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        
if __name__ == '__main__':
    # ===================== TO BE REMOVED ===================== 
    import warnings
    warnings.filterwarnings("ignore")
    # ===================== TO BE REMOVED =====================

    some_name_node = DashedLineDetector(node_name='dashed_line_detection_node')
    rospy.spin()