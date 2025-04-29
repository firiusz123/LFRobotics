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
        self.hue_min_slider.set(0)
        self.hue_min_slider.pack(side="left")

        self.hue_max_slider = Scale(slider_frame, from_=0, to_=255, orient=HORIZONTAL, label="Hue Max", command=self.update_filter)
        self.hue_max_slider.set(180)
        self.hue_max_slider.pack(side="left")

        self.sat_min_slider = Scale(slider_frame, from_=0, to_=255, orient=HORIZONTAL, label="Saturation Min", command=self.update_filter)
        self.sat_min_slider.set(0)
        self.sat_min_slider.pack(side="left")

        self.sat_max_slider = Scale(slider_frame, from_=0, to_=255, orient=HORIZONTAL, label="Saturation Max", command=self.update_filter)
        self.sat_max_slider.set(255)
        self.sat_max_slider.pack(side="left")

        self.val_min_slider = Scale(slider_frame, from_=0, to_=255, orient=HORIZONTAL, label="Value Min", command=self.update_filter)
        self.val_min_slider.set(0)
        self.val_min_slider.pack(side="left")

        self.val_max_slider = Scale(slider_frame, from_=0, to_=255, orient=HORIZONTAL, label="Value Max", command=self.update_filter)
        self.val_max_slider.set(255)
        self.val_max_slider.pack(side="left")

        # Toggle button to switch between showing binary mask and masked image
        self.show_binary_mask_button = Button(self.root, text="Show Binary Mask", command=self.toggle_mask_view)
        self.show_binary_mask_button.pack(side="bottom")

        # Invert mask button
        self.invert_mask_button = Button(self.root, text="Invert Mask", command=self.invert_mask)
        self.invert_mask_button.pack(side="bottom")

        # Save image button
        self.save_button = Button(self.root, text="Save Image", command=self.save_image)
        self.save_button.pack(side="bottom")

        # Store the image, filtered image, and binary mask
        self.cv_image = None
        self.filtered_image = None
        self.binary_mask = None
        self.show_binary_mask = False

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.update_filter()
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)

    def update_filter(self, event=None):
        if self.cv_image is not None:
            hue_min = self.hue_min_slider.get()
            hue_max = self.hue_max_slider.get()
            sat_min = self.sat_min_slider.get()
            sat_max = self.sat_max_slider.get()
            val_min = self.val_min_slider.get()
            val_max = self.val_max_slider.get()

            hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

            hue_min_scaled = int(hue_min * 180 / 255)
            hue_max_scaled = int(hue_max * 180 / 255)

            lower_bound = np.array([hue_min_scaled, sat_min, val_min], dtype=np.uint8)
            upper_bound = np.array([hue_max_scaled, sat_max, val_max], dtype=np.uint8)

            self.binary_mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            self.filtered_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.binary_mask)

            self.display_images()

    def display_images(self):
        if self.cv_image is not None:
            original_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            original_image_pil = Image.fromarray(original_image)
            original_image_tk = ImageTk.PhotoImage(original_image_pil)

            self.image_label_original.config(image=original_image_tk)
            self.image_label_original.image = original_image_tk

        if self.show_binary_mask and self.binary_mask is not None:
            display_image = cv2.cvtColor(self.binary_mask, cv2.COLOR_GRAY2RGB)
        elif self.filtered_image is not None:
            display_image = cv2.cvtColor(self.filtered_image, cv2.COLOR_BGR2RGB)
        else:
            return

        display_image_pil = Image.fromarray(display_image)
        display_image_tk = ImageTk.PhotoImage(display_image_pil)

        self.image_label_filtered.config(image=display_image_tk)
        self.image_label_filtered.image = display_image_tk

    def toggle_mask_view(self):
        self.show_binary_mask = not self.show_binary_mask
        self.show_binary_mask_button.config(
            text="Show Masked Image" if self.show_binary_mask else "Show Binary Mask"
        )
        self.display_images()

    def invert_mask(self):
        if self.binary_mask is not None:
            self.binary_mask = cv2.bitwise_not(self.binary_mask)
            self.filtered_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.binary_mask)
            self.display_images()

    def save_image(self):
        if self.cv_image is None:
            rospy.logwarn("No image to save.")
            return

        if self.show_binary_mask and self.binary_mask is not None:
            image_to_save = self.binary_mask
            filename = "saved_binary_mask.png"
        elif self.filtered_image is not None:
            image_to_save = self.filtered_image
            filename = "saved_filtered_image.png"
        else:
            rospy.logwarn("No processed image to save.")
            return

        success = cv2.imwrite(filename, image_to_save)
        if success:
            rospy.loginfo(f"Image saved as {filename}")
        else:
            rospy.logerr("Failed to save image.")

def main():
    rospy.init_node('image_viewer', anonymous=True)
    root = Tk()
    viewer = ImageViewer(root)
    root.mainloop()

if __name__ == '__main__':
    main()
