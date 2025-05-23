#!/usr/bin/env python3
import rospy

import numpy as np
import cv_bridge
import cv2
# import tensorflow as tf
import subprocess
# import DTROS-related class
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

# import messages and services
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage

from model import SimpleNet
import torch
class NNController(DTROS):
    def __init__(self, node_name):
        super(NNController, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )

        # Set maximum linear speed
        self.v_max = DTParam('~v_max', param_type=ParamType.FLOAT)

        # Image gausian blur mask
        self.image_blur_ksize = DTParam('~image_blur_ksize', param_type=ParamType.DICT)

        # Image crop 
        self.image_crop = DTParam('~image_crop', param_type=ParamType.DICT)

        # Image resize
        self.image_resize =  DTParam('~image_resize', param_type=ParamType.DICT)

        # Neural controller model name
        self.nn_model_path = DTParam('~nn_model_path', param_type=ParamType.STRING)
        self.nn_model_name = DTParam('~nn_model_name', param_type=ParamType.STRING)
        
        rospy.loginfo(self.nn_model_path.value + '/' + self.nn_model_name.value)

        self.cvbridge = cv_bridge.CvBridge()

        # Subscribe to image topic
        self.sub_image = rospy.Subscriber('~in/image/compressed', CompressedImage, self.callback, queue_size=1)
        
        # Publishers
        self.pub_control = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
        self.pub_debug_img = rospy.Publisher('~debug/image/compressed', CompressedImage, queue_size=1)
    

        # Model NN
        # A - Place your code here
        # subprocess.run(['pwd'])

        self.twist = Twist2DStamped()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.load('/code/catkin_ws/src/Main/packages/controllers/config/skillissue2.pth' , weights_only=False)
        self.model.to(self.device)
        self.model.eval()
        # Message to publish


    def callback(self, msg) -> None:
        try:

            # Read image from node
            img_org = self.cvbridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Image processing            
            # Crop image
            # image = img_org[self.image_crop.value['start']:self.image_crop.value['stop'], :, :]
            image = img_org
            # Convert the image to a PyTorch tensor (with proper channels)
            image_tensor = torch.from_numpy(image).float()

            # Normalize image for NN (optional, depending on how your model expects the data)
            # image_tensor = image_tensor / 255.0  # If needed, normalize here
            
            # Add batch dimension to image tensor (PyTorch expects a batch dimension as the first dim)
            image_tensor = image_tensor.permute(2,0,1)  # Shape becomes (1, 3, height, width)
            # rospy.loginfo(f"image size  {image_tensor.shape}")
            # Predict control omega
            if self.model:
                with torch.no_grad():
                    self.twist.omega = self.model(image_tensor.to(self.device)).item()*24
            # if self.model:
            #     # with self.model.eval():
            #     self.twist.omega = self.model(torch.tensor(image,dtype=torch.float32))
            if abs(self.twist.omega) > 0.5:
                rospy.loginfo(f"Omega {self.twist.omega}")
            # Set linear speed
            self.twist.v = self.v_max.value
            
            # Set timestamp
            self.twist.header.stamp = rospy.Time.now()
            
            # Publish control
            self.pub_control.publish(self.twist)
            
            # DEBUG
            if self.pub_debug_img.anybody_listening():
                # Publish transformed image
                # Add error value to image
                cv2.putText(img_org, 
                    "Controll= " + str( "%.3f" % self.twist.omega), 
                    org=(10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0,255,0), fontScale=0.5, 
                    thickness=1, lineType=cv2.LINE_AA)
                out_image = self.cvbridge.cv2_to_compressed_imgmsg(img_org, 'jpg')
                out_image.header.stamp = rospy.Time.now()
                # self.pub_debug_img.publish(out_image)

        except Exception as e:
            rospy.logerr("Error: {0}".format(e))
    
    def on_switch_on(self):
        rospy.loginfo("Neurnal Network controller switched from off to on")

    def on_switch_off(self): # The node will shut down before this code is run, so it's effectively useless, but it reminds us of inevitable death upon which we can take no action and after which we will serve no purpose
        pass 

    def on_shutdown(self):
        # Send stop command
        for _ in range(10000):
            self.pub_control.publish(Twist2DStamped(omega=0.0, v=0.0))
        rospy.sleep(1)
        self.pub_control.publish(Twist2DStamped(omega=0.0, v=0.0))
        rospy.sleep(1)
        self.pub_control.publish(Twist2DStamped(omega=0.0, v=0.0))
        # Wait....
        rospy.sleep(1)
        rospy.loginfo("Stop NN controller")

###################################################################################

if __name__ == '__main__':
    some_name_node = NNController(node_name='nn_controller_node')
    rospy.spin()
