#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from tkinter import Tk, Label, Scale, HORIZONTAL, Button, Frame
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from PIL import Image, ImageTk

class ImageViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS Image Viewer with HSV Filter")

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Create labels for original and filtered images
        self.image_label_original = Label(self.root)
        self.image_label_original.pack(side="left")

        self.image_label_filtered = Label(self.root)
        self.image_label_filtered.pack(side="right")

        # Set up the ROS subscriber for the compressed image topic
        self.image_sub = rospy.Subscriber("/d3/camera_node/image/compressed", CompressedImage, self.image_callback)

        # Frame for sliders
        slider_frame = Frame(self.root)
        slider_frame.pack(side="bottom")

        # HSV Filter sliders (min and max for each channel)
        self.hue_min_slider = Scale(slider_frame, from_=0, to_=255, orient=HORIZONTAL, label="Hue Min", command=self.update_filter)
        self.hue_min_slider.set(0)  # Set default value to 0
        self.hue_min_slider.pack(side="left")
        self.hue_max_slider = Scale(slider_frame, from_=0, to_=255, orient=HORIZONTAL, label="Hue Max", command=self.update_filter)
        self.hue_max_slider.set(180)  # Set default value to 180
        self.hue_max_slider.pack(side="left")

        self.sat_min_slider = Scale(slider_frame, from_=0, to_=255, orient=HORIZONTAL, label="Saturation Min", command=self.update_filter)
        self.sat_min_slider.set(0)  # Set default value to 0
        self.sat_min_slider.pack(side="left")
        self.sat_max_slider = Scale(slider_frame, from_=0, to_=255, orient=HORIZONTAL, label="Saturation Max", command=self.update_filter)
        self.sat_max_slider.set(255)  # Set default value to 255
        self.sat_max_slider.pack(side="left")

        self.val_min_slider = Scale(slider_frame, from_=0, to_=255, orient=HORIZONTAL, label="Value Min", command=self.update_filter)
        self.val_min_slider.set(0)  # Set default value to 0
        self.val_min_slider.pack(side="left")
        self.val_max_slider = Scale(slider_frame, from_=0, to_=255, orient=HORIZONTAL, label="Value Max", command=self.update_filter)
        self.val_max_slider.set(255)  # Set default value to 255
        self.val_max_slider.pack(side="left")

        # Toggle button to switch between showing binary mask and masked image
        self.show_binary_mask_button = Button(self.root, text="Show Binary Mask", command=self.toggle_mask_view)
        self.show_binary_mask_button.pack(side="bottom")

        # Invert mask button
        self.invert_mask_button = Button(self.root, text="Invert Mask", command=self.invert_mask)
        self.invert_mask_button.pack(side="bottom")

        # Store the image, filtered image, and binary mask
        self.cv_image = None
        self.filtered_image = None
        self.binary_mask = None
        self.show_binary_mask = False

    def image_callback(self, msg):
        # Convert the compressed ROS image message to OpenCV format
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.update_filter()  # Update filter on image arrival
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)

    def update_filter(self, event=None):
        if self.cv_image is not None:
            # Get the HSV filter values from the sliders
            hue_min = self.hue_min_slider.get()
            hue_max = self.hue_max_slider.get()
            sat_min = self.sat_min_slider.get()
            sat_max = self.sat_max_slider.get()
            val_min = self.val_min_slider.get()
            val_max = self.val_max_slider.get()

            # Convert the image to HSV
            hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

            # Scale hue values to match 0-255 range
            hue_min_scaled = int(hue_min * 180 / 255)
            hue_max_scaled = int(hue_max * 180 / 255)

            # Define the lower and upper bounds for the HSV filter
            lower_bound = np.array([hue_min_scaled, sat_min, val_min], dtype=np.uint8)
            upper_bound = np.array([hue_max_scaled, sat_max, val_max], dtype=np.uint8)

            # Mask the image with the HSV filter
            self.binary_mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            self.filtered_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.binary_mask)

            self.display_images()

    def display_images(self):
        # Convert the original image to a format that Tkinter can handle
        if self.cv_image is not None:
            original_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            original_image_pil = Image.fromarray(original_image)
            original_image_tk = ImageTk.PhotoImage(original_image_pil)

            self.image_label_original.config(image=original_image_tk)
            self.image_label_original.image = original_image_tk  # Keep a reference to the image

        # Convert the image to be shown based on button state
        if self.show_binary_mask and self.binary_mask is not None:
            display_image = cv2.cvtColor(self.binary_mask, cv2.COLOR_GRAY2RGB)
        elif self.filtered_image is not None:
            display_image = cv2.cvtColor(self.filtered_image, cv2.COLOR_BGR2RGB)
        else:
            return

        # Convert the selected image to Tkinter format
        display_image_pil = Image.fromarray(display_image)
        display_image_tk = ImageTk.PhotoImage(display_image_pil)

        self.image_label_filtered.config(image=display_image_tk)
        self.image_label_filtered.image = display_image_tk  # Keep a reference to the image

    def toggle_mask_view(self):
        # Toggle between showing binary mask and masked image
        self.show_binary_mask = not self.show_binary_mask
        # Update the button text based on the current view
        if self.show_binary_mask:
            self.show_binary_mask_button.config(text="Show Masked Image")
        else:
            self.show_binary_mask_button.config(text="Show Binary Mask")

        self.display_images()

    def invert_mask(self):
        # Invert the current binary mask
        if self.binary_mask is not None:
            self.binary_mask = cv2.bitwise_not(self.binary_mask)
            self.filtered_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.binary_mask)
            self.display_images()

def main():
    rospy.init_node('image_viewer', anonymous=True)
    
    # Set up Tkinter window
    root = Tk()
    
    # Initialize the ImageViewer class
    viewer = ImageViewer(root)
    
    # Start the Tkinter main loop
    root.mainloop()

if __name__ == '__main__':
    main()
