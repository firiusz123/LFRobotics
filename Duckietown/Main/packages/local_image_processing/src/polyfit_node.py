#!/usr/bin/env python3

import rospy # type: ignore
import cv2
import cv_bridge # type: ignore
import threading
import numpy as np
from math import sqrt

# import DTROS-related classes
from duckietown.dtros import DTROS,DTParam,NodeType,ParamType # type: ignore

# import messages and services
from sensor_msgs.msg import CompressedImage # type: ignore
from std_msgs.msg import Float32MultiArray, Float32 # type: ignore

class Polyfit(DTROS):

    def __init__(self, node_name):
        super(Polyfit, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        

        # Desired point parameters
        self.point_param = DTParam("~follow_point", param_type=ParamType.DICT)
        
        # Normalization values for error
        self.normalizer_param = DTParam("~normalizer", param_type=ParamType.DICT)
        
        # Angle threshold to determine if the points lay on a straight line or should a 3rd degree polynomial be used instead
        self.angle_treshold = 10

        self.pub_error = rospy.Publisher('~image/error', Float32, queue_size=1)

        # Max degree of polynomial to be calculated
        self.poly_degree = DTParam('~degree', param_type=ParamType.INT).value
        
        self.centers = []
        self.points = []
        self.error_buffer_size = DTParam('~buffer_size', param_type=ParamType.INT).value
        self.error_buffer = [None] * self.error_buffer_size
        self.mean_error = 0
        
        self.error = {'raw' : None, 'norm' : None}

        # Subscriber to centroids topic that contains centroid points
        self.sub_points = rospy.Subscriber('~image/centroids', Float32MultiArray, self.callback, queue_size=1)
        
        # Publishers
        self.pub_error = rospy.Publisher('~image/error' , Float32 , queue_size = 1)
        
        # Poly function and desired horizontal line that should return desired point on polynomial
        self.polyFunction = None
        self.zeroPoint = [ self.point_param.value["pointX"],self.point_param.value["pointY"] ]
        
        # Normalization values
        self.max = self.normalizer_param.value["maxValue"]
        self.min = self.normalizer_param.value["minValue"]


    def polyfit(self):

        def notNone(obj):
            return obj is not None

        # as default we can assume degree of 3 
        poly_degree = self.poly_degree
        threshold_triggered = False

        # Filter out None points
        centers = [pt for pt in self.points if notNone(pt)]
        if len(centers) < 2:
            return

        # Separete points into x and y coordinates so that it works with polynomial fitting
        x_points, y_points = zip(*centers)
        centers_length = len(centers)
        try: 
            poly_degree = 3
            if centers_length > 3:
                for i in range(len(centers) - 1):
                    v1 = (x_points[i], y_points[i])
                    v2 = (x_points[i+1] - x_points[i], y_points[i+1] - y_points[i])
                    mag1 = np.hypot(*v1)
                    mag2 = np.hypot(*v2)
                    if mag1 == 0 or mag2 == 0:
                        return
                    angle_rad = np.arccos(np.clip((v1[0]*v2[0] + v1[1]*v2[1]) / (mag1 * mag2), -1.0, 1.0))
                    angle_deg = np.degrees(angle_rad)
                    if angle_deg <= self.angle_treshold:
                        poly_degree = 1
                        break
            elif len(centers) == 2:
                poly_degree = 1
            else:
                poly_degree = 2
 
        except Exception as e:
            rospy.logerr(e)
            return 
        


        timeout_event = threading.Event()

        def fit_poly():
            try:
                np.seterr(all='ignore')
                cheb_poly = np.polynomial.chebyshev.Chebyshev.fit(
                    x_points, y_points, deg=poly_degree, domain=[min(x_points), max(x_points)]
                )
                poly_standard = cheb_poly.convert(kind=np.polynomial.Polynomial)
                self.polyFunction = np.poly1d(poly_standard.coef[::-1])
            except Exception as e:
                rospy.logerr(f"Polyfit failed: {e}")
            finally:
                timeout_event.set()
        
        try:
            polyfit_thread = threading.Thread(target=fit_poly)
            polyfit_thread.start()
            if not timeout_event.wait(timeout=0.01):
                rospy.logwarn("Polyfit computation timed out")
            else:
                polyfit_thread.join()
        except Exception as e:
            rospy.logerr(f"Error in threading: {e}")

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
            # Get the points data, in a form of an array [(x_0,y_0), (x_1,y_1), ...]
            merged_points = msg.data
            x, y = merged_points[::2], merged_points[1::2]
            x, y = list(x),list(y)
            self.points = list(zip(merged_points[::2], merged_points[1::2]))
            
            # Fit the best curve through the points
            self.polyfit()
            if self.polyFunction == None or len(self.points) == 0 or self.points == None:
                msg1 = Float32()
                msg1.data = self.error['norm'] if self.error['norm'] != None else 0
                # msg1.header.stamp = rospy.Time.now()
                self.pub_error.publish(msg1)
                pass
            
            if self.polyFunction != None:

                # Calculate intersection points
                intersectionXValues = (self.polyFunction - self.zeroPoint[1]).roots.real
                intersectionPoints = [ (x,self.zeroPoint[1]) for x in intersectionXValues ]

                # Get the closest point from the chunk cetnters points to the desired y line
                # This could be prone to some problems it there were two yellow pieces of the road on each side
                # But then again - even the polyfitting falls apart in that case
                closestLinePoint = self.select_closest_point_y(self.points,self.zeroPoint[1])
                
                # Get the closest point from intersection points to the one calculated previously
                extendedLinePoint = self.select_closest_point_reference(intersectionPoints,closestLinePoint)
                
                candidateError = float('inf')
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
                
                # Publish transformed image
                if abs(self.error['norm']) > 1:
                    self.error['norm'] = self.mean_error
                if self.error['norm']:
                    msg1 = Float32()
                    msg1.data = self.error['norm']
                    self.pub_error.publish(msg1)
                
                notNone_values = 0
                self.mean_error = 0
                self.median_error = None
                if None in self.error_buffer: 
                    for i in range(self.error_buffer_size):
                        if self.error_buffer[i] != None:
                            notNone_values+=1
                            self.mean_error+=self.error_buffer[i]
                    if notNone_values != 0:
                        self.mean_error = self.mean_error/notNone_values
                    else:
                        self.mean_error = 0
                else:
                    self.error_buffer.sort()
                    self.median_error = self.error_buffer[self.error_buffer_size//2]
                
                # rospy.loginfo(f"Normalized:\t{self.error['norm']} Median:\t{self.median_error} Mean:\t{self.mean_error}")

                msg1 = Float32()
                # msg1.data = self.median_error if self.median_error else self.mean_error
                
                if self.error['norm'] is not None:
                    msg1.data = self.error['norm']
                    # msg1.header.stamp = rospy.Time.now()
                    self.pub_error.publish(msg1)

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        
if __name__ == '__main__':

    # ===================== TO BE REMOVED ===================== 
    import warnings
    warnings.filterwarnings("ignore")
    # ===================== TO BE REMOVED =====================

    some_name_node = Polyfit(node_name='dashed_line_detection_node')
    rospy.spin()


    # Old polyfit to inspection
    # def polyfit(self):
    #     def notNone(object):
    #         return object is not None
        
    #     # as deafault we can assume degree of 3 
    #     poly_degree = 3
    #     threshold_triggered = 0

    #     filteredCenters = filter(notNone, self.points)
    #     x_points, y_points = [], []
    #     centers = []
    #     for element in filteredCenters:
    #         centers.append(element)
    #     if len(centers) and len(centers) >2 :
    #         x_points, y_points = zip(*centers)
            
    #         for i in range(len(centers) - 1): 
    #             v1 = (x_points[i] , y_points[i])
    #             v2 = (x_points[1+1] - x_points[i]   , y_points[i+1] - y_points[i])
    #             dot = v1[0] * v2[0] + v1[1] * v2[1]
    #             mag1 = np.sqrt(v1[0]**2 + v1[1]**2)
    #             mag2 = np.sqrt(v2[0]**2 + v2[1]**2)
    #             if mag1 == 0 or mag2 == 0:
    #                 return 0  
    #             angle_rad = np.arccos(dot / (mag1 * mag2))
    #             angle_deg = np.degrees(angle_rad)
    #             if angle_deg <= 3:
    #                 threshold_triggered = 1
    #             else:
    #                 threshold_triggered = 0

    #     if not threshold_triggered:
    #         poly_degree = 3 
    #     else:
    #         poly_degree = 1
    #     # Create an event to
    #     #for center in centers:  handle the timeout
    #     timeout_event = threading.Event()

    #     def fit_poly():
    #         try:
    #             np.seterr(all='ignore')  # Ignore warnings
    #             poly = np.polyfit(x_points, y_points, poly_degree)
    #             self.polyFunction = np.poly1d(poly)
    #             timeout_event.set()  # Mark the event as finished
    #         except Exception as e:
    #             rospy.loginfo(e)
    #             pass

    #     # Start the thread to run polyfit
    #     polyfit_thread = threading.Thread(target=fit_poly)
    #     polyfit_thread.start()

    #     # Wait for the thread to complete or timeout after 0.3 seconds
    #     if not timeout_event.wait(timeout=0.05):
    #         rospy.loginfo("Polyfit computation timed out, leaving polyFunction unchanged")
    #         # Optionally, you can also terminate the thread if needed
    #         # However, Python does not have a clean way to kill threads directly, so it is better to let them finish naturally
    #     else:
    #         # rospy.loginfo("Polyfit computation completed successfully.")
    #         polyfit_thread.join()  
