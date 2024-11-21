import cv2
import numpy as np
import yaml
import os

# Helper function to load YAML configuration
def load_yaml_config(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

# Function to load the selected color configuration
def load_color_config(color_name):
    color_config_path = f"../config/lateral_position_error_node/color/{color_name}.yaml"  # Path to the color configuration file
    color_data = load_yaml_config(color_config_path)
    color_mask = {
        'lower1': np.array(color_data['color']['lower1']),
        'upper1': np.array(color_data['color']['upper1']),
        'lower2': np.array(color_data['color']['lower2']),
        'upper2': np.array(color_data['color']['upper2'])
    }
    return color_mask

# Load image parameters (width, height) and search area (top, bottom) from default.yaml
image_param_path = "../config/lateral_position_error_node/default.yaml"
image_data = load_yaml_config(image_param_path)

image_param = {
    'width': int(image_data['image_param']['width']),
    'height': int(image_data['image_param']['height']),
}

search_area_data = image_data['search_area']
search_area = {
    'top': int(search_area_data['top']),
    'bottom': int(search_area_data['bottom']),
}

# Load in image (replace with your image path)
image = cv2.imread('../../../assets/rqt/image1.png')

# Select the color to track, e.g., "red", "white", or "yellow"
color_name = "yellow"  # Change to 'white' or 'yellow' as needed
color_line_mask = load_color_config(color_name)

print(color_line_mask)

# Create a window
cv2.namedWindow('image')

# Convert image to HSV color space
image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Find follow line (color detection)
lower_mask = cv2.inRange(image, color_line_mask['lower1'], color_line_mask['upper1'])
upper_mask = cv2.inRange(image, color_line_mask['lower2'], color_line_mask['upper2'])

# Combine both masks
full_mask = lower_mask + upper_mask

# Mask image
result_mask = cv2.bitwise_and(image, image, mask=full_mask)

# Cut image, only consider the search area
result_mask = result_mask[search_area['top']:search_area['bottom'], 0:image_param['width']]

gray_image = cv2.cvtColor(result_mask, cv2.COLOR_BGR2GRAY)

_, thresh_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

# Find contours of the object in the thresholded image
contours, _ = cv2.findContours(thresh_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Assuming the largest contour is the object
# Get the moments of the largest contour
largest_contour = max(contours, key=cv2.contourArea)

# Calculate moments for the largest contour
M = cv2.moments(largest_contour)

# Check if the moment is valid to avoid division by zero
if M['m00'] != 0:
    # Calculate the center of mass (centroid)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    print(f"Center of Mass (Centroid): ({cx}, {cy})")
else:
    cx, cy = None, None
    print("No object found.")

# Visualize the centroid on the image
if cx is not None and cy is not None:
    cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1) 

# Display output image

cv2.imshow('image', image)
cv2.imshow('lower', lower_mask)
# cv2.imshow('upper', upper_mask)
cv2.imshow('image_hsv', image_hsv)
cv2.imshow('full_mask', full_mask)
cv2.imshow('result_mask',result_mask)

# Wait for a key press
cv2.waitKey(0)
cv2.destroyAllWindows()
