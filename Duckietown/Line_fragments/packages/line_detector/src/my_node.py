import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image (ensure the file is uploaded and path is correct)
image_path = '/home/student/Documents/LFRobotics/Duckietown/Line_fragments/packages/line_detector/src/ngur.png'  # Change this path to your image location
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


def detected_line_using_red_result(red_result, threshold=0.3):
    # Convert the red_result to grayscale to count non-zero intensities
    grayscale_red = cv2.cvtColor(red_result, cv2.COLOR_BGR2GRAY)

    # Count the number of non-zero pixels
    non_zero_pixels = np.count_nonzero(grayscale_red)

    # Calculate the total number of pixels in the image
    total_pixels = grayscale_red.size

    # Calculate the percentage of non-zero pixels (red intensity detected)
    red_percentage = non_zero_pixels / total_pixels

    # Return 1 if red intensity is at least the threshold percentage, otherwise return 0
    return 1 if red_percentage >= threshold else 0

# Example usage with red_result
result = detected_line_using_red_result(red_result, threshold=0.15)
print(f"Red detected in at least 10% of the image: {'Yes' if result == 1 else 'No'}")
