import cv2
import numpy as np

# ===== Color Ranges =====
ORANGE_LOWER = np.array([5, 120, 120])
ORANGE_UPPER = np.array([22, 255, 255])
BLUE_LOWER   = np.array([95, 80, 60])
BLUE_UPPER   = np.array([125, 255, 255])

RED_LOWER1   = np.array([0, 150, 80])
RED_UPPER1   = np.array([10, 255, 255])
RED_LOWER2   = np.array([170,150,80])
RED_UPPER2   = np.array([179,255,255])

GREEN_LOWER  = np.array([40, 80, 80])
GREEN_UPPER  = np.array([85, 255, 255])

MAGENTA_LOWER= np.array([140, 80, 80])
MAGENTA_UPPER= np.array([170, 255, 255])

# ===== Lane Detection =====
def find_lane_offset(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_o = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
    mask_b = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    mask = cv2.bitwise_or(mask_o, mask_b)
    h, w = mask.shape
    roi = mask[int(h*0.5):, :]
    M = cv2.moments(roi)
    if M['m00'] == 0:
        return None
    cx = int(M['m10'] / M['m00'])
    return (cx - w/2) / (w/2)

# ===== Pillar Detection =====
def detect_pillars(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1) | cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
    return red_mask, green_mask

def pillar_centroid(mask):
    h, w = mask.shape
    roi = mask[int(h*0.4):, :]
    M = cv2.moments(roi)
    if M['m00'] == 0:
        return None
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00']) + int(h*0.4)
    return (cx, cy)

# ===== Parking Marker Detection =====
def find_parking_marker(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, MAGENTA_LOWER, MAGENTA_UPPER)
    M = cv2.moments(mask)
    if M['m00'] == 0:
        return None
    return int(M['m10']/M['m00'])
