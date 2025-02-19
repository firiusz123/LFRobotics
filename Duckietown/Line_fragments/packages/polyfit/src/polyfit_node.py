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
        

        # Desired point parameters
        self.point_param = DTParam("~follow_point", param_type=ParamType.DICT)
        
        # Normalization values for error
        self.normalizer_param = DTParam("~normalizer", param_type=ParamType.DICT)
        
        self.angle_treshold = 10

        # Max degree of polynomial to be calculated
        self.poly_degree = DTParam('~degree', param_type=ParamType.DICT).value
        
        self.centers = []
        self.points = []

        # Poly function and desired horizontal line that should return desired point on polynomial
        self.polyFunction = None
        self.zeroPoint = [ self.point_param.value["pointX"],self.point_param.value["pointY"] ]
        
        # Normalization values
        self.max = self.normalizer_param.value["maxValue"]
        self.min = self.normalizer_param.value["minValue"]
        

        self.error = {'raw' : None, 'norm' : None}

        # Subscriber to centroids topic that contains centroid points
        self.sub_image = rospy.Subscriber('~image/centroids', Float32MultiArray, self.callback, queue_size=1)
        
        # Publishers
        self.error_pub = rospy.Publisher('~image/error' , Float32 , queue_size = 1)
        
    def polyfit(self):
        def notNone(object):
            return object is not None
        # as deafault we can assume degree of 3 
        poly_degree = self.poly_degree
        threshold_triggered = 0

        filteredCenters = filter(notNone, self.points)
        if len(self.points) == 0:
            return
        x_points, y_points = [], []
        centers = []
        for element in filteredCenters:
            centers.append(element)
        centers_length = len(centers)
        if centers_length and centers_length > 2 :
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
                if angle_deg <= self.angle_treshold:
                    threshold_triggered = 1
                else:
                    threshold_triggered = 0
        elif centers_length == 2:
            x_points, y_points = zip(*centers)
            threshold_triggered = 1
        else:
            return
        if not threshold_triggered:
            poly_degree = self.poly_degree
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
                rospy.loginfo(f"Centeres: {centers} x_points: {x_points} y_points: {y_points} Degree {poly_degree} ")
                rospy.logerr(e)
                pass

        # Start the thread to run polyfit
        polyfit_thread = threading.Thread(target=fit_poly)
        polyfit_thread.start()
        # Wait for the thread to complete or timeout after 0.01 seconds
        if not timeout_event.wait(timeout=0.01):
            rospy.loginfo("Polyfit computation timed out, leaving polyFunction unchanged")
            # Optionally, you can also terminate the thread if needed
            # However, Python does not have a clean way to kill threads directly, so it is better to let them finish naturally
        else:
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



    
    
    
    def callback(self, msg) -> None:
        try:       
            # Read input image

            # self.points = msg.data
            
            # Calculate centers of line strips

            # DEBUG
            merged_points = msg.data
            x, y = merged_points[::2], merged_points[1::2]
            x, y = list(x),list(y)
            self.points = list(zip(merged_points[::2], merged_points[1::2]))
            self.polyfit()
            if self.polyFunction == None or len(self.points) == 0 or self.points == None:
                msg1 = Float32()
                msg1.data = self.error['norm']
                # msg1.header.stamp = rospy.Time.now()
                self.error_pub.publish(msg1)
                return
            # if self.pub_debug_img.anybody_listening():
            # Calculate intersection points
            intersectionXValues = (self.polyFunction - self.zeroPoint[1]).roots.real
            intersectionPoints = [ (x,self.zeroPoint[1]) for x in intersectionXValues ]
            # Get all of the green point from line fragments to
             
            closestLinePoint = self.select_closest_point_y(self.points,self.zeroPoint[1])
            # Get the closest point from intersection points to the one calculated previously
            extendedLinePoint = self.select_closest_point_reference(intersectionPoints,closestLinePoint)
            
            #for intersection in intersectionXValues:
            #    cv2.circle(image, (int(intersection), self.zeroPoint[1]), 10, (0,0,255), -1)
                        
            candidateError = float('inf')
            # for intersectionX in intersectionXValues:
            #     candidateError = min(candidateError,(self.zeroPoint[0] - max(intersectionXValues))).real
            candidateError = self.zeroPoint[0] - extendedLinePoint[0]
            self.error['raw'] = candidateError
            # Due to problems with PID, the values of min and max will be fixed
            # if self.error['raw'] > self.max:
            #     self.max = self.error['raw']
            # if self.error['raw'] < self.min:
            #     self.min = self.error['raw']
            if self.max != self.min:
                norm = 2 * (self.error['raw'] - self.min) / (self.max - self.min) - 1
            else:
                norm = 0  # Or any default normalized value
            self.error['norm'] = norm.real
            # rospy.loginfo(f"Max value: {self.max} Min value: {self.min} Value of candidate error: {candidateError}")
            # Publish transformed image
            # rospy.loginfo(f"Publishing {self.error['norm']}")
            msg1 = Float32()
            msg1.data = self.error['norm']
            # msg1.header.stamp = rospy.Time.now()
            self.error_pub.publish(msg1)
            # rospy.loginfo(msg1)
        except cv_bridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        
if __name__ == '__main__':
    # ===================== TO BE REMOVED ===================== 
    import warnings
    warnings.filterwarnings("ignore")
    # ===================== TO BE REMOVED =====================

    some_name_node = DashedLineDetector(node_name='dashed_line_detection_node')
    rospy.spin()