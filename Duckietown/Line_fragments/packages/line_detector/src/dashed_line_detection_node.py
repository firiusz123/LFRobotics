#!/usr/bin/env python3


import rospy
import cv2
import cv_bridge
import threading
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
from std_msgs.msg import Float32MultiArray, Float32
from scipy.interpolate import splprep, splev

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
        self.zeroPoint = [100, 380]
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
    
    def fit_spline(self, points):
        """
        Fits a parametric spline to the given points.
        """
        if not points:
            return [], []
        
        points_array = np.array(points)
        x_points, y_points = points_array[:, 0], points_array[:, 1]

        # Parameterize the curve using t
        tck, u = splprep([x_points, y_points], s=0, k=3)

        # Generate the smooth curve
        u_fine = np.linspace(0, 1, 1000)  # Fine-grained parameter values
        x_smooth, y_smooth = splev(u_fine, tck)

        return x_smooth, y_smooth
    
    import numpy as np

    def fit_linear(self, points):
        # Assuming points is a list or array of (x, y) pairs
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]
        
        # Fit a line: y = mx + b
        m, b = np.polyfit(x, y, 1)  # Fit a line (degree 1 polynomial)
        
        # Generate the smoothed points
        y_smooth = m * x + b
        return x, y_smooth

    
    def callback(self, msg) -> None:
        try:
            # Read input image
            image = self.cvbridge.compressed_imgmsg_to_cv2(msg)

            height, width = image.shape[:2]
            image_cropped = image[height // 2:, :]  # Keep the bottom half of the image

            # Apply Gaussian Blur to smooth the cropped image
            blurred_image = cv2.GaussianBlur(image_cropped, (41, 41), 0)

            # Convert both original and blurred images to HSV color space
            hsv_image_not_blurred = cv2.cvtColor(image_cropped, cv2.COLOR_BGR2HSV)
            hsv_image_blurred = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

            # Define a more moderate HSV range for detecting yellow
            lower_yellow = np.array([21, 106, 0])  # Lower bound for yellow in HSV
            upper_yellow = np.array([100, 255, 255])  # Upper bound for yellow in HSV

            # Create masks for the yellow color range
            yellow_mask_not_blurred = cv2.inRange(hsv_image_not_blurred, lower_yellow, upper_yellow)
            yellow_mask_blurred = cv2.inRange(hsv_image_blurred, lower_yellow, upper_yellow)

            # Combine both masks using bitwise_and
            combined_yellow_mask = cv2.bitwise_and(yellow_mask_not_blurred, yellow_mask_blurred)

            # Apply the combined mask to the original image
            yellow_result = cv2.bitwise_and(image_cropped, image_cropped, mask=combined_yellow_mask)

            # Convert yellow_result to grayscale (for contour detection)
            gray_yellow_result = cv2.cvtColor(yellow_result, cv2.COLOR_BGR2GRAY)

            # Find contours on the grayscale image (or binary mask)
            contours, _ = cv2.findContours(gray_yellow_result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Copy yellow_result to draw contours
            yellow_contours = np.copy(yellow_result)

            # Draw the contours on the yellow_result
            cv2.drawContours(yellow_contours, contours, -1, (0, 255, 0), 2)  # Green contours

            # Draw the contours and their centers of mass on yellow_result
            yellow_centroids = np.copy(yellow_result)

            self.points = []

            for con in contours:
                M = cv2.moments(con)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    self.points.append((cx, cy))  # Append centroid
                    cv2.circle(yellow_centroids, (cx, cy), 5, (0, 0, 255), -1)  # Red circle

            self.points = [(x, y + height // 2) for x, y in self.points]
            
            if len(self.points) > 3:
                # Use spline fitting instead of poly function
                x_smooth, y_smooth = self.fit_spline(self.points)
            else:
                rospy.logwarn("Not enough points to fit spline. Fitting a linear curve instead.")
                x_smooth, y_smooth = self.fit_linear(self.points)

            # Calculate closest point using spline-fitted curve
            x, y = list(x_smooth), list(y_smooth)

            # Publish detected points (no poly function involved)
            msg_x = Float32MultiArray()
            msg_x.data = x
            msg_y = Float32MultiArray()
            msg_y.data = y
            self.points_pub['x'].publish(msg_x)
            self.points_pub['y'].publish(msg_y)

            # Inside the callback method where spline fitting is done

            if self.pub_debug_img.anybody_listening():
                # Draw centroids on the image
                for i, center in enumerate(self.points):
                    if center:
                        cv2.circle(image, (int(center[0]), int(center[1])), 10, ((i*20)%256, 255, 0), -1)

                # Draw the spline curve directly using x_smooth and y_smooth
                draw_points = np.array([x_smooth, y_smooth]).T.astype(np.int32)  # Points from the spline
                cv2.polylines(image, [draw_points], False, color=(255, 0, 0), thickness=2)  # Draw spline line

                # # Calculate error (raw and normalized)
                # intersection_points = [(x, self.zeroPoint[1]) for x in x_smooth if np.isclose(y_smooth[np.isclose(x_smooth, x)], self.zeroPoint[1])]

                # Calculate the absolute differences between y_smooth and self.zeroPoint[1]
                distances = np.abs(y_smooth - self.zeroPoint[1])

                # Find the index of the minimum distance
                closest_index = np.argmin(distances)

                # Select the intersection point corresponding to the closest index
                closestLinePoint = (x_smooth[closest_index], self.zeroPoint[1])

                # Calculate intersection points
                # indices = np.where(np.isclose(y_smooth, self.zeroPoint[1]))[0]

                # # Create intersection points using the indices
                # intersection_points = [(x_smooth[i], self.zeroPoint[1]) for i in indices]

                # # closestLinePoint = self.select_closest_point_y(intersection_points, self.zeroPoint[1])
                # closestLinePoint = intersection_points[0]
                # # extendedLinePoint = self.select_closest_point_reference(intersection_points, closestLinePoint)

                # Draw additional points
                cv2.circle(image, (int(self.zeroPoint[0]), int(self.zeroPoint[1])), 10, (255, 0, 255), -1)
                # if extendedLinePoint is not None:
                #     cv2.circle(image, (int(extendedLinePoint[0]), int(extendedLinePoint[1])), 10, (255, 255, 255), -1)
                # else:
                #     rospy.logwarn("Extended line point is None, skipping drawing.")
                cv2.circle(image, (int(closestLinePoint[0]), int(closestLinePoint[1])), 15, (255, 0, 0), -1)

                # Calculate raw error
                candidateError = self.zeroPoint[0] - closestLinePoint[0]
                self.error['raw'] = candidateError

                # Update max and min error for normalization
                if self.error['raw'] > self.max:
                    self.max = self.error['raw']
                if self.error['raw'] < self.min:
                    self.min = self.error['raw']

                # Normalize error
                if self.max != self.min:
                    norm = 2 * (self.error['raw'] - self.min) / (self.max - self.min) - 1
                else:
                    norm = 0  # Or any default normalized value

                self.error['norm'] = norm.real

                # Publish error
                rospy.loginfo(f"Publishing data {self.error['raw']} {self.error['norm']}")
                self.pub_error['raw'].publish(self.error['raw'])
                self.pub_error['norm'].publish(self.error['norm'])

                # Add error value to image
                cv2.rectangle(image, (0, 0), (self.image_param.value['width'], 50), (255, 255, 255), -1)
                cv2.putText(image, "Points : " + str(self.error['norm']),
                            org=(10, 20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0, 0, 0), fontScale=0.5,
                            thickness=1, lineType=cv2.LINE_AA)

                # Publish transformed image
                debug_out_image = self.cvbridge.cv2_to_compressed_imgmsg(np.concatenate(([image]), axis=1).reshape(
                    (self.image_param.value['height'], self.image_param.value['width'], 3)), 'jpg')
                debug_out_image.header.stamp = rospy.Time.now()

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