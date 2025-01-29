import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# Load the image (ensure the file path is correct)
image_path = "/home/student/Documents/LFRobotics/Duckietown/Line_fragments/packages/line_detector/src/ngur3.png"  # Replace with your image file name or path
image = cv2.imread(image_path)

if image is None:
    print("Error: Image not found or failed to load.")
    exit()

# Apply Gaussian Blur to smooth the image
# Crop the top half of the image
height, width = image.shape[:2]
image_cropped = image[height // 2:, :]  # Keep the bottom half of the image

# Apply Gaussian Blur to smooth the cropped image
blurred_image = cv2.GaussianBlur(image_cropped, (41, 41), 0)

# Convert both original and blurred images to HSV color space
hsv_image_not_blurred = cv2.cvtColor(image_cropped, cv2.COLOR_BGR2HSV)
hsv_image_blurred = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)


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

# Create a list to store the points (centroids)
points = []

for con in contours:
    M = cv2.moments(con)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        points.append((cx, cy))  # Append centroid
        cv2.circle(yellow_centroids, (cx, cy), 5, (0, 0, 255), -1)  # Red circle

print("Points (centroids):", points)

# Fit a parametric spline to the centroids
def fit_spline(points):
    # Convert points to numpy array
    points_array = np.array(points)
    x_points, y_points = points_array[:, 0], points_array[:, 1]

    # Parameterize the curve using t
    tck, u = splprep([x_points, y_points], s=0, k=3)

    # Generate the smooth curve
    u_fine = np.linspace(0, 1, 1000)  # Fine-grained parameter values
    x_smooth, y_smooth = splev(u_fine, tck)

    return x_smooth, y_smooth

points = [(x, y + height // 2) for x, y in points]

# Fit the spline
x_smooth, y_smooth = fit_spline(points)

# Define the horizontal line y = 100
y_intersection = 650

# Find the point(s) on the spline where y = 100
intersection_points = []
for x, y in zip(x_smooth, y_smooth):
    if abs(y - y_intersection) < 1:  # Allow a small tolerance
        intersection_points.append((x, y))

# Print the intersection points
print(f"Intersection points with y = {y_intersection}:", intersection_points)


# Display images using matplotlib
plt.figure(figsize=(8, 6))
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title("Original Image")
plt.axis('off')
plt.show()

plt.figure(figsize=(8, 6))
plt.imshow(cv2.cvtColor(blurred_image, cv2.COLOR_BGR2RGB))
plt.title("Blurred Image")
plt.axis('off')
plt.show()

plt.figure(figsize=(8, 6))
plt.imshow(combined_yellow_mask, cmap='gray')
plt.title("Combined Yellow Mask")
plt.axis('off')
plt.show()

plt.figure(figsize=(8, 6))
plt.imshow(cv2.cvtColor(yellow_result, cv2.COLOR_BGR2RGB))
plt.title("Yellow Result")
plt.axis('off')
plt.show()

plt.figure(figsize=(8, 6))
plt.imshow(cv2.cvtColor(yellow_contours, cv2.COLOR_BGR2RGB))
plt.title("Yellow Contours")
plt.axis('off')
plt.show()

plt.figure(figsize=(8, 6))
plt.imshow(cv2.cvtColor(yellow_centroids, cv2.COLOR_BGR2RGB))
plt.title("Yellow Centroids")
plt.axis('off')
plt.show()

# Now show the fitted spline curve
plt.figure(figsize=(8, 6))
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.plot(x_smooth, y_smooth, label="Fitted Spline", color='blue', linewidth=2)
plt.scatter(*zip(*points), color='red', label="Centroids", zorder=5)  # Plot centroids
plt.title("Spline Fit on Centroids with Original Image")
plt.axis('off')
plt.legend()
plt.show()
