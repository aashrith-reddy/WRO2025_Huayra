import cv2
import numpy as np
from picamera2 import Picamera2

# ===== Color Ranges =====
COLOR_RANGES = {
    "orange":  (np.array([5, 120, 120]),  np.array([22, 255, 255])),
    "blue":    (np.array([95, 80, 60]),   np.array([125, 255, 255])),
    "red1":    (np.array([0, 150, 80]),   np.array([10, 255, 255])),
    "red2":    (np.array([170,150,80]),   np.array([179,255,255])),
    "green":   (np.array([40, 80, 80]),   np.array([85, 255, 255])),
    "magenta": (np.array([140, 80, 80]),  np.array([170, 255, 255]))
}

# ===== Initialize PiCamera2 =====
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Press 'q' to quit")

while True:
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Loop through colors and detect
    for color, (lower, upper) in COLOR_RANGES.items():
        if color == "red1":  
            # Special handling for red (two ranges)
            mask1 = cv2.inRange(hsv, COLOR_RANGES["red1"][0], COLOR_RANGES["red1"][1])
            mask2 = cv2.inRange(hsv, COLOR_RANGES["red2"][0], COLOR_RANGES["red2"][1])
            mask = cv2.bitwise_or(mask1, mask2)
            display_color = (0, 0, 255)
            label = "red"
        else:
            mask = cv2.inRange(hsv, lower, upper)
            if color == "orange":  display_color = (0, 140, 255)
            elif color == "blue": display_color = (255, 0, 0)
            elif color == "green": display_color = (0, 255, 0)
            elif color == "magenta": display_color = (255, 0, 255)
            label = color

        # Find contours for each color
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:  # filter noise
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), display_color, 2)
                cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, display_color, 2)
    
    # Show camera feed
    cv2.imshow("Color Detection", frame)
    
    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
