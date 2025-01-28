import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image (ensure the file is uploaded and path is correct)
image_path = 'Line_fragments/packages/line_detector/src/ngur2.png'  # Change this path to your image location
image = cv2.imread(image_path)

if image is None:
    print("Error: Image not found or failed to load.")
    exit()

# Crop the top half of the image
height, width, _ = image.shape

# Apply Gaussian Blur to smooth the cropped image
blurred_image = cv2.GaussianBlur(image, (31, 31), 0)
cv2.imwrite("blurred_image.png", blurred_image)  # Save blurred image
print("Saved blurred_image.png")

# Convert the blurred image to HSV color space
hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

# Define a more moderate HSV range for detecting yellow
lower_yellow = np.array([25, 120, 120])  # Lower bound for yellow in HSV
upper_yellow = np.array([40, 255, 255])  # Upper bound for yellow in HSV

# Create a mask for the yellow color range
yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
cv2.imwrite("yellow_mask.png", yellow_mask)  # Save yellow mask
print("Saved yellow_mask.png")

# Apply the mask to the original (not blurred) image for clear yellow regions
yellow_result = cv2.bitwise_and(image, image, mask=yellow_mask)
cv2.imwrite("yellow_result.png", yellow_result)  # Save yellow result
print("Saved yellow_result.png")

# Convert yellow_result to grayscale (for contour detection)
gray_yellow_result = cv2.cvtColor(yellow_result, cv2.COLOR_BGR2GRAY)
cv2.imwrite("gray_yellow_result.png", gray_yellow_result)  # Save grayscale yellow result
print("Saved gray_yellow_result.png")

# Find contours on the grayscale image (or binary mask)
contours, _ = cv2.findContours(gray_yellow_result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Copy yellow_result to draw contours
yellow_contours = np.copy(yellow_result)

# Draw the contours on the yellow_result
cv2.drawContours(yellow_contours, contours, -1, (0, 255, 0), 2)  # Green contours
cv2.imwrite("yellow_contours.png", yellow_contours)  # Save contours image
print("Saved yellow_contours.png")

# Draw the contours and their centers of mass on the yellow_result
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

cv2.imwrite("yellow_centroids.png", yellow_centroids)  # Save centroids image
print("Saved yellow_centroids.png")

# Now, we apply polyfit to the points
def polyfit(points, degree=3):
    
    # Separate x and y coordinates
    x_points, y_points = zip(*points)

    # Fit polynomial to the points
    poly = np.polyfit(x_points, y_points, degree)
    poly_function = np.poly1d(poly)

    return poly_function, x_points, y_points

# Fit polynomial to the centroids
poly_function, x_points, y_points = polyfit(points, degree=3)

if poly_function:
    print("Polynomial function:", poly_function)

    # Generate x values for the fitted curve
    x_range = np.linspace(min(x_points), max(x_points), 1000)

    # Generate the corresponding y values using the polynomial
    y_range = poly_function(x_range)

    # Plot the fitted polynomial on the original image
    plt.figure(figsize=(8, 6))
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.scatter(x_points, y_points, color='red', label="Centroids", zorder=5)
    plt.plot(x_range, y_range, label=f"Polyfit (Degree 3)", color='blue', linewidth=2, zorder=6)
    plt.title("Polyfit on Centroids")
    plt.legend()
    plt.axis('off')
    plt.show()

# Display all images using matplotlib
plt.figure(figsize=(12, 8))

# Original Image
plt.subplot(231)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title("Original Image")
plt.axis('off')

# Blurred Image
plt.subplot(232)
plt.imshow(cv2.cvtColor(blurred_image, cv2.COLOR_BGR2RGB))
plt.title("Blurred Image")
plt.axis('off')

# Yellow Mask
plt.subplot(233)
plt.imshow(yellow_mask, cmap='gray')
plt.title("Yellow Mask")
plt.axis('off')

# Yellow Result
plt.subplot(234)
plt.imshow(cv2.cvtColor(yellow_result, cv2.COLOR_BGR2RGB))
plt.title("Yellow Result")
plt.axis('off')

# Yellow Contours
plt.subplot(235)
plt.imshow(cv2.cvtColor(yellow_contours, cv2.COLOR_BGR2RGB))
plt.title("Yellow Contours")
plt.axis('off')

# Yellow Centroids
plt.subplot(236)
plt.imshow(cv2.cvtColor(yellow_centroids, cv2.COLOR_BGR2RGB))
plt.title("Yellow Contours with Centroids")
plt.axis('off')

plt.tight_layout()
plt.show()

# Print final output message with file names
print("All images saved as PNG files. Check the directory.")
