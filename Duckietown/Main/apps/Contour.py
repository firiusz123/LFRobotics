#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ImageSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_display_node', anonymous=True)
        # Subscribe to the compressed image topic
        self.image_sub = rospy.Subscriber("/d3/camera_node/image/compressed", CompressedImage, self.image_callback)
    
    def image_callback(self, data):
        try:
            # Decode the compressed image
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Display the image using OpenCV
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        img_sub = ImageSubscriber()
        img_sub.run()
    except rospy.ROSInterruptException:
        pass
