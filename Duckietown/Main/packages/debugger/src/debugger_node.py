#!/usr/bin/env python3


import rospy
import cv2
import cv_bridge
import threading
import numpy as np
from math import sqrt
# import matplotlib.pyplot as plt  

# import DTROS-related classes
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

# import messages and services
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Float32
import message_filters

class Debugger(DTROS):

    def __init__(self, node_name):
        super(Debugger, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        self.centers = []
        self.points = []
        self.polyFunction = None
        self.point_param = DTParam("~follow_point", param_type=ParamType.DICT)
        self.zeroPoint = [self.point_param.value["pointX"],self.point_param.value["pointY"]]
        # Read color mask  
        rospy.loginfo("debugger started here it comes")
        
        # self.figure, self.ax = plt.subplot()

        # Camera parameters
        self.image_param = DTParam('~image_param', param_type=ParamType.DICT)
        self.max = self.image_param.value["width"]-self.zeroPoint[0]
        self.min = -self.zeroPoint[0]
        
        # Subscribe to image topic
        self.sub_image = message_filters.Subscriber('~image/mask/compressed', CompressedImage,queue_size=1)
        self.sub_centroids = message_filters.Subscriber('~image/centroids', Float32MultiArray, queue_size=1)
        self.sub_polyerror = message_filters.Subscriber('~image/error' , Float32 , queue_size = 1)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_image, self.sub_centroids, self.sub_polyerror],queue_size=1, slop=0.5, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        self.cvbridge = cv_bridge.CvBridge()

        # Transformed image
        self.pub_debug_img = rospy.Publisher('~image/debugger_out/out/compressed', CompressedImage, queue_size=1)

        

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

        # Wait for the thread to complete or timeout after 0.01 seconds
        if not timeout_event.wait(timeout=0.01):
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
    
    

    
    def callback(self, image , centroids , error) -> None:
        try:           
            self.points = centroids.data
            image = self.cvbridge.compressed_imgmsg_to_cv2(image)
            self.angleThreshold = 1

            if self.points:
            # DEBUG
                x, y = [], []
                x, y = self.points[::2], self.points[1::2]
                x, y = list(x),list(y)
                self.points = list(zip(self.points[::2], self.points[1::2]))
                self.polyfit()
                # if self.pub_debug_img.anybody_listening()
                for i,center in enumerate(self.points):
                    if center:
                        cv2.circle(image, (int(center[0]), int(center[1])), 10, ((i*20)%256,255,0), -1)
                draw_x = np.linspace(min(x) , max(x) , int(max(x)-min(x)))
                #draw_x = np.linspace(0 , 640, 640)
            
            if self.polyFunction and self.points:
                draw_y = self.polyFunction(draw_x)
                
                draw_points = (np.asarray([draw_x, draw_y]).T).astype(np.int32) 
                # line, = self.ax.plot(draw_x,draw_y)
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
          
            

            #cv2.circle(image, (int(self.xpoint), int(self.polyFunction(self.xpoint))), 10, (255,0,0), -1)
                cv2.polylines(image, [draw_points], False,color=(255,0,0) , thickness= 2) 
            # Add error value to image
            cv2.rectangle(image, (0, 0), (self.image_param.value['width'], 50), (255,255,255), -1)
            cv2.putText(image, "Points : " + str(error), 
                org=(10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0,0,0), fontScale=0.5, 
                thickness=1, lineType=cv2.LINE_AA)
            debug_out_image = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([image]),axis=1).reshape(
                (self.image_param.value['height'], self.image_param.value['width'], 3)), 'jpg')
            debug_out_image.header.stamp = rospy.Time.now()
            
            # Publish transformed image

            self.pub_debug_img.publish(debug_out_image)
            # rospy.loginfo("debbuger callback triggered")

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        
if __name__ == '__main__':
    # ===================== TO BE REMOVED ===================== 
    import warnings
    warnings.filterwarnings("ignore")
    # ===================== TO BE REMOVED =====================
    some_name_node = Debugger(node_name='debugger_node')
    rospy.spin()