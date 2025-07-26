import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

# Extended HSV range for blue (light to dark)
lower_blue = np.array([90, 50, 50])
upper_blue = np.array([140, 255, 255])

# HSV range for orange
lower_orange = np.array([10, 100, 20])
upper_orange = np.array([25, 255, 255])

while True:
    frame = picam2.capture_array()

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Create color masks
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

    # Combine blue and orange masks
    combined_mask = cv2.bitwise_or(mask_blue, mask_orange)

    # Apply the combined mask to the original frame
    result = cv2.bitwise_and(frame, frame, mask=combined_mask)

    # Show only the detected colors
    cv2.imshow("Detected Blue and Orange", result)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
