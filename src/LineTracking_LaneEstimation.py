import cv2
import numpy as np
from picamera2 import Picamera2


REAL_LINE_THICKNESS_MM = 20.0   # WRO line thickness
FRAME_SIZE = (640, 480)         # capture size
MIN_CONTOUR_AREA = 2000         # ignore small blobs
SMOOTH_KERNEL = 5               # blur kernel for noise reduction

FOCAL_LENGTH_PX = None  # Calibrate with 'c' key

# HSV ranges for orange and blue
HSV_RANGES = {
    "orange": (
        np.array([5, 120, 120]),
        np.array([25, 255, 255])
    ),
    "blue": (
        np.array([95, 80, 60]),
        np.array([135, 255, 255])
    )
}

# Drawing colors
DRAW_COLORS = {
    "orange": (0, 165, 255),
    "blue": (255, 0, 0),
    "lane_center": (0, 255, 255)
}

def find_largest_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0
    largest = max(contours, key=cv2.contourArea)
    return largest, cv2.contourArea(largest)

def oriented_bbox_and_thickness(contour):
    rect = cv2.minAreaRect(contour)
    (cx, cy), (rw, rh), ang = rect
    rw, rh = float(rw), float(rh)
    if rw == 0 or rh == 0:
        return (int(cx), int(cy)), (int(rw), int(rh)), ang, 0
    perceived_thickness = min(rw, rh)
    return (int(cx), int(cy)), (int(round(rw)), int(round(rh))), ang, perceived_thickness

def pixel_to_mm_lateral(offset_px, distance_mm, focal_px):
    return (offset_px * distance_mm) / focal_px

def estimate_distance_mm(real_width_mm, perceived_px, focal_px):
    if perceived_px <= 0 or focal_px is None:
        return None
    return (real_width_mm * focal_px) / perceived_px

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": FRAME_SIZE})
picam2.configure(config)
picam2.start()
print("Camera started. Press 'q' to quit, 'c' to calibrate focal length.")

while True:
    frame = picam2.capture_array()
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    blurred = cv2.GaussianBlur(frame_bgr, (SMOOTH_KERNEL, SMOOTH_KERNEL), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    detections = {}

    for color_name, (lower, upper) in HSV_RANGES.items():
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))

        contour, area = find_largest_contour(mask)
        if contour is None or area < MIN_CONTOUR_AREA:
            continue

        center, (rw, rh), angle, thickness_px = oriented_bbox_and_thickness(contour)
        x, y, w, h = cv2.boundingRect(contour)

        dist_mm = None
        if FOCAL_LENGTH_PX is not None and thickness_px > 0:
            dist_mm = estimate_distance_mm(REAL_LINE_THICKNESS_MM, thickness_px, FOCAL_LENGTH_PX)

        detections[color_name] = {
            "center": center,
            "thickness_px": thickness_px,
            "distance_mm": dist_mm,
            "area": area,
            "bbox": (x, y, w, h)
        }

        color_draw = DRAW_COLORS.get(color_name, (200, 200, 200))
        cv2.rectangle(frame_bgr, (x, y), (x+w, y+h), color_draw, 2)
        cv2.circle(frame_bgr, center, 4, color_draw, -1)
        label = f"{color_name} t={thickness_px:.1f}px"
        if dist_mm is not None:
            label += f", d={dist_mm:.0f}mm"
        cv2.putText(frame_bgr, label, (x, max(y-8, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_draw, 2)

    closest_color = None
    if detections:
        distances = [(c, d["distance_mm"]) for c, d in detections.items() if d["distance_mm"] is not None]
        if distances:
            closest_color = min(distances, key=lambda x: x[1])[0]
        else:
            closest_color = max(detections.items(), key=lambda kv: (kv[1]["thickness_px"], kv[1]["area"]))[0]

    lane_center_px = None
    lateral_offset_px = None
    lateral_offset_mm = None
    distance_to_first_line_mm = None

    if "orange" in detections and "blue" in detections:
        oc = detections["orange"]["center"]
        bc = detections["blue"]["center"]
        lane_center_px = ((oc[0] + bc[0]) // 2, (oc[1] + bc[1]) // 2)
        img_center = (FRAME_SIZE[0] // 2, FRAME_SIZE[1] // 2)
        lateral_offset_px = lane_center_px[0] - img_center[0]

        if detections["orange"]["distance_mm"] is not None and detections["blue"]["distance_mm"] is not None:
            first_line_color = "orange" if detections["orange"]["distance_mm"] < detections["blue"]["distance_mm"] else "blue"
        else:
            first_line_color = "orange" if detections["orange"]["thickness_px"] > detections["blue"]["thickness_px"] else "blue"

        distance_to_first_line_mm = detections[first_line_color]["distance_mm"]

        if FOCAL_LENGTH_PX is not None and distance_to_first_line_mm is not None:
            lateral_offset_mm = pixel_to_mm_lateral(lateral_offset_px, distance_to_first_line_mm, FOCAL_LENGTH_PX)

        cv2.line(frame_bgr, oc, bc, DRAW_COLORS["lane_center"], 2)
        cv2.circle(frame_bgr, lane_center_px, 5, DRAW_COLORS["lane_center"], -1)
        cv2.putText(frame_bgr, f"Lane offset: {lateral_offset_px}px", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, DRAW_COLORS["lane_center"], 2)

        if lateral_offset_mm is not None:
            cv2.putText(frame_bgr, f"~{lateral_offset_mm:.0f} mm", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, DRAW_COLORS["lane_center"], 2)

    elif closest_color and detections.get(closest_color):
        distance_to_first_line_mm = detections[closest_color]["distance_mm"]
        cv2.putText(frame_bgr, f"{closest_color} closest", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, DRAW_COLORS[closest_color], 2)

    if closest_color:
        info = f"Closest color: {closest_color}"
        if distance_to_first_line_mm is not None:
            info += f", distance: {distance_to_first_line_mm:.0f} mm"
        print(info)

    if FOCAL_LENGTH_PX is None:
        cv2.putText(frame_bgr, "Focal length: NOT CALIBRATED (press 'c')", (10, FRAME_SIZE[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
    else:
        cv2.putText(frame_bgr, f"Focal length: {FOCAL_LENGTH_PX:.1f}px", (10, FRAME_SIZE[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

    cv2.imshow("Lane Detection", frame_bgr)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

    if key == ord('c'):
        KNOWN_DISTANCE_MM = float(input("Enter known distance to the line in mm: "))
        if closest_color and detections.get(closest_color):
            perceived_px = detections[closest_color]["thickness_px"]
            if perceived_px > 0:
                FOCAL_LENGTH_PX = (perceived_px * KNOWN_DISTANCE_MM) / REAL_LINE_THICKNESS_MM
                print(f"Calibrated focal length = {FOCAL_LENGTH_PX:.2f} px")
            else:
                print("Calibration failed: perceived thickness is zero.")
        else:
            print("No line detected for calibration.")

picam2.stop()
cv2.destroyAllWindows()
