import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image (ensure the file is uploaded and path is correct)
image_path = 'Line_fragments/packages/line_detector/src/ngur.png'  # Change this path to your image location
image = cv2.imread(image_path)

if image is None:
    print("Error: Image not found or failed to load.")
    exit()

# Crop the top half of the image
height, width, _ = image.shape
cropped_image = image[height // 2:, :]  # Retain only the bottom half

# Apply Gaussian Blur to smooth the cropped image
blurred_image = cv2.GaussianBlur(cropped_image, (21, 21), 0)

# Convert the blurred image to HSV color space
hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

# Define the HSV range for detecting red
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])

# Create masks for the red color ranges
mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
red_mask = mask1 | mask2

# Apply the mask to the blurred image to extract red regions
red_result = cv2.bitwise_and(blurred_image, blurred_image, mask=red_mask)

# Save the images to files (optional, for inspection)
cv2.imwrite("original_image.png", image)
cv2.imwrite("cropped_image.png", cropped_image)
cv2.imwrite("blurred_image.png", blurred_image)
cv2.imwrite("red_masked_image.png", red_result)

# Display all images using matplotlib
plt.figure(figsize=(12, 8))

# Original Image
plt.subplot(231)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title("Original Image")
plt.axis('off')

# Cropped Image
plt.subplot(232)
plt.imshow(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))
plt.title("Cropped Image")
plt.axis('off')

# Blurred Image
plt.subplot(233)
plt.imshow(cv2.cvtColor(blurred_image, cv2.COLOR_BGR2RGB))
plt.title("Blurred Image")
plt.axis('off')

# Red Masked Image
plt.subplot(234)
plt.imshow(cv2.cvtColor(red_result, cv2.COLOR_BGR2RGB))
plt.title("Red Masked Image")
plt.axis('off')

plt.tight_layout()
plt.show()

print("Images saved as PNG files. Check the directory.")
