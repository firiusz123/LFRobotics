import cv2
import sys
import numpy as np

# Load in image
image = cv2.imread('../../../assets/image1.png')

# Create a window
cv2.namedWindow('image')

# Display output image
cv2.imshow('image',output)

# Wait longer to prevent freeze for videos.
if 0xFF == ord('q'):
    cv2.destroyAllWindows()