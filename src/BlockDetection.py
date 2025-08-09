import cv2
import numpy as np
import math
from picamera2 import Picamera2

# =====================
# Known data for blocks
# =====================
BLOCKS = {
    "Red": {
        "rgb": (238, 39, 55),
        "width_mm": 50,  # real-world width in mm
    },
    "Green": {
        "rgb": (68, 214, 44),
        "width_mm": 50,
    },
    "Magenta": {
        "rgb": (255, 0, 255),
        "width_mm": 200,
    }
}

# Convert RGB to HSV range for color detection
def rgb_to_hsv_range(rgb):
    color_bgr = np.uint8([[rgb[::-1]]])  # convert RGB to BGR
    hsv_color = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)[0][0]

    # A simple range for detection (+/- 10 hue, +/- 50 saturation/value)
    lower_bound = np.array([max(0, hsv_color[0] - 10), max(50, hsv_color[1] - 50), max(50, hsv_color[2] - 50)])
    upper_bound = np.array([min(179, hsv_color[0] + 10), min(255, hsv_color[1] + 50), min(255, hsv_color[2] + 50)])
    return lower_bound, upper_bound

# Precompute HSV ranges
for name, data in BLOCKS.items():
    BLOCKS[name]["hsv_range"] = rgb_to_hsv_range(data["rgb"])

# =====================
# Camera calibration
# =====================
KNOWN_DISTANCE_MM = 300  # Distance from camera for calibration
KNOWN_PIXEL_WIDTH = 100  # Measured pixel width at KNOWN_DISTANCE_MM

focal_length = (KNOWN_PIXEL_WIDTH * KNOWN_DISTANCE_MM) / BLOCKS["Red"]["width_mm"]

def calculate_distance(real_width_mm, pixel_width):
    if pixel_width == 0:
        return None
    return (real_width_mm * focal_length) / pixel_width

# =====================
# Initialize Pi Camera
# =====================
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.start()

while True:
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    detected_blocks = []

    for color_name, data in BLOCKS.items():
        lower, upper = data["hsv_range"]
        mask = cv2.inRange(hsv, lower, upper)

        # Remove noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 500:  # filter out tiny objects
                x, y, w, h = cv2.boundingRect(largest_contour)
                distance_mm = calculate_distance(data["width_mm"], w)
                detected_blocks.append({
                    "color": color_name,
                    "distance": distance_mm,
                    "area": area,
                    "box": (x, y, w, h)
                })

    # Find block with the largest area
    if detected_blocks:
        closest_block = max(detected_blocks, key=lambda b: b["area"])
        x, y, w, h = closest_block["box"]

        # Draw rectangle & label
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        label = f"{closest_block['color']} - {closest_block['distance']:.1f} mm"
        cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Print info to terminal
        print(f"Closest Block: {closest_block['color']} | Distance: {closest_block['distance']:.1f} mm")

    cv2.imshow("Block Distance Measurement", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
