#!/usr/bin/env python3
"""
Integrated obstacle-round control (current version)
- Keeps your exact hardware initialization block
- Picamera2 for vision (rotated 180 deg)
- HC-SR04 ultrasonic sensors (trig/echo) with per-sensor max distance 3m
- Largest contour detection (red/green/blue/orange). Only largest shown.
- Collision avoidance (cases 1..6), center align nudges, lane alignment,
  corner turns (blue/orange) and block pass maneuvers (red/green).
"""

import time
from time import sleep, time as now
import cv2
import numpy as np

# Hardware imports
from gpiozero import DigitalOutputDevice
from rpi_hardware_pwm import HardwarePWM
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# ======================================================
# HARDWARE INITIALIZATION (KEEP EXACTLY AS PROVIDED)
# ======================================================

# Motor + Steering
direction = DigitalOutputDevice(23)                    # Motor direction pin
pwm = HardwarePWM(pwm_channel=0, hz=18000, chip=0)    # Motor PWM (rear motor)
servo = HardwarePWM(pwm_channel=1, hz=50, chip=0)     # Steering servo

# Start values
direction.value = 0
pwm.start(0)        # Motor stopped (set forward later)
servo.start(7.5)    # Neutral steering position

# Ultrasonic sensor pins (PLACEHOLDERS, replace with real pin numbers)
ULTRASONIC_PINS = {
    "front": {"TRIG": 26, "ECHO": 16, "max_distance_m": 3},
    "left":  {"TRIG": 17, "ECHO": 27, "max_distance_m": 3},
    "right": {"TRIG": 5,  "ECHO": 6,  "max_distance_m": 3},
}

# GPIO Setup
GPIO.setmode(GPIO.BCM)
for sensor, pins in ULTRASONIC_PINS.items():
    # Only configure pins if not placeholder zeros
    if pins["TRIG"] != 0:
        GPIO.setup(pins["TRIG"], GPIO.OUT)
        GPIO.output(pins["TRIG"], GPIO.LOW)
    if pins["ECHO"] != 0:
        GPIO.setup(pins["ECHO"], GPIO.IN)

# Camera setup (kept similar to previous)
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# ======================================================
# PARAMETERS (all tunables)
# ======================================================
COLLISION_THRESHOLD = 15.0  # cm: immediate collision trigger
ALIGN_THRESHOLD = 30.0      # cm: center-align band upper bound
STEERING_STEP = 0.3         # servo correction step for small nudges (duty offset)
STEERING_NEUTRAL = 7.5      # neutral servo duty
MOTOR_SPEED = 15            # default forward PWM % (forward always uses this)
REDUCED_PWM = 11            # reduced motor % used for cautious moves / nudges

REVERSE_DUTY = 10           # reverse PWM %
REVERSE_TIME_S = 0.10       # reverse duration (100 ms)

# Contour and vision tuning
FRAME_SIZE = (640, 480)
MIN_AREA = {
    "blue": 1000,
    "orange": 1000,
    "green": 3000,
    "red": 3000
}
BLOCK_PASS_AREA_THRESHOLD = 13000  # when block is close enough to trigger pass maneuver

# Maneuver offsets/times (servo units are duty cycle)
CORNER_TURN_OFFSET = 1.5   # servo duty offset for corner turn (NEUTRAL +/- this)
CORNER_TURN_TIME = 0.50    # seconds for corner turn (500ms)
CORNER_STABILIZE = 0.30    # seconds to stabilize after corner

PASS_STRONG_OFFSET = 0.5   # strong sidestep offset for passing
PASS_STRONG_TIME = 0.50    # 500 ms
PASS_CORRECTION_OFFSET = 0.3
PASS_CORRECTION_TIME = 0.20

# Loop timing
LOOP_DELAY = 0.005

# Round direction variable
round_color = None  # will store "blue" or "orange" once first corner observed

# HSV ranges (placeholders; tune later)
HSV_BLUE_LOWER = np.array([95, 80, 60])
HSV_BLUE_UPPER = np.array([135, 255, 255])
HSV_ORANGE_LOWER = np.array([5, 120, 120])
HSV_ORANGE_UPPER = np.array([25, 255, 255])
HSV_GREEN_LOWER = np.array([40, 70, 70])
HSV_GREEN_UPPER = np.array([85, 255, 255])
HSV_RED1_LOWER = np.array([0, 100, 100])
HSV_RED1_UPPER = np.array([10, 255, 255])
HSV_RED2_LOWER = np.array([160, 100, 100])
HSV_RED2_UPPER = np.array([179, 255, 255])

# ======================================================
# UTILITY: Servo & motor primitives
# ======================================================
def clamp_servo(duty):
    return max(5.0, min(9.5, duty))  # per your hardware limits

def set_servo_duty(duty):
    servo.change_duty_cycle(clamp_servo(duty))

def steer_neutral():
    set_servo_duty(STEERING_NEUTRAL)

def steer_left_strong():
    # leftmost is smaller duty
    set_servo_duty(5.0)

def steer_right_strong():
    set_servo_duty(9.5)

def drive_forward(pct=MOTOR_SPEED):
    direction.value = 0
    pwm.change_duty_cycle(pct)

def drive_reverse(pct=REVERSE_DUTY):
    direction.value = 1
    pwm.change_duty_cycle(pct)

def stop_drive():
    pwm.change_duty_cycle(0)

# ======================================================
# UTILITY: Ultrasonic measurement (HC-SR04)
# ======================================================
def measure_hcsr04(trig_pin, echo_pin, max_distance_cm):
    """Measure distance (cm) with HC-SR04. Returns measured distance in cm,
       or max_distance_cm on timeout or invalid reading."""
    if trig_pin == 0 or echo_pin == 0:
        return max_distance_cm

    # timeout based on max distance
    max_time = max_distance_cm / 17150.0  # seconds

    # trigger pulse
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    t0 = time.time()
    # wait for echo HIGH
    while GPIO.input(echo_pin) == 0:
        if time.time() - t0 > max_time:
            return max_distance_cm
    start = time.time()

    # wait for echo LOW
    while GPIO.input(echo_pin) == 1:
        if time.time() - start > max_time:
            return max_distance_cm
    end = time.time()

    pulse = end - start
    # distance in cm
    dist_cm = pulse * 17150.0
    # clamp
    if dist_cm > max_distance_cm:
        return max_distance_cm
    return dist_cm

def read_all_distances():
    """Return dict {'left':cm, 'front':cm, 'right':cm}"""
    d = {}
    for name in ("left", "front", "right"):
        cfg = ULTRASONIC_PINS[name]
        max_d_cm = cfg.get("max_distance_m", 3) * 100
        d[name] = measure_hcsr04(cfg["TRIG"], cfg["ECHO"], max_d_cm)
    return d

# ======================================================
# VISION HELPERS
# ======================================================
def to_hsv(frame):
    return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

def largest_contour_in_mask(mask, min_area):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, 0
    best = max(cnts, key=cv2.contourArea)
    return best, cv2.contourArea(best)

def detect_largest_block(frame):
    """Return (best_color, area, bbox(cx,cy,w,h)) or (None,0,None)"""
    hsv = to_hsv(frame)

    # red special: combine masks
    red_mask1 = cv2.inRange(hsv, HSV_RED1_LOWER, HSV_RED1_UPPER)
    red_mask2 = cv2.inRange(hsv, HSV_RED2_LOWER, HSV_RED2_UPPER)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    green_mask = cv2.inRange(hsv, HSV_GREEN_LOWER, HSV_GREEN_UPPER)
    blue_mask = cv2.inRange(hsv, HSV_BLUE_LOWER, HSV_BLUE_UPPER)
    orange_mask = cv2.inRange(hsv, HSV_ORANGE_LOWER, HSV_ORANGE_UPPER)

    candidates = []

    # red
    cnt, area = largest_contour_in_mask(red_mask, MIN_AREA["red"])
    if cnt is not None and area >= MIN_AREA["red"]:
        x,y,w,h = cv2.boundingRect(cnt)
        candidates.append(("red", area, (x,y,w,h)))

    # green
    cnt, area = largest_contour_in_mask(green_mask, MIN_AREA["green"])
    if cnt is not None and area >= MIN_AREA["green"]:
        x,y,w,h = cv2.boundingRect(cnt)
        candidates.append(("green", area, (x,y,w,h)))

    # blue
    cnt, area = largest_contour_in_mask(blue_mask, MIN_AREA["blue"])
    if cnt is not None and area >= MIN_AREA["blue"]:
        x,y,w,h = cv2.boundingRect(cnt)
        candidates.append(("blue", area, (x,y,w,h)))

    # orange
    cnt, area = largest_contour_in_mask(orange_mask, MIN_AREA["orange"])
    if cnt is not None and area >= MIN_AREA["orange"]:
        x,y,w,h = cv2.boundingRect(cnt)
        candidates.append(("orange", area, (x,y,w,h)))

    if not candidates:
        return None, 0, None

    # choose largest by area among candidates
    best = max(candidates, key=lambda x: x[1])
    return best  # (color, area, bbox)

def detect_corner_line(frame):
    """Return 'blue' or 'orange' if a corner line is confidently detected, else None."""
    hsv = to_hsv(frame)
    blue_mask = cv2.inRange(hsv, HSV_BLUE_LOWER, HSV_BLUE_UPPER)
    orange_mask = cv2.inRange(hsv, HSV_ORANGE_LOWER, HSV_ORANGE_UPPER)
    # use largest_contour_in_mask to check presence & area
    _, b_area = largest_contour_in_mask(blue_mask, MIN_AREA["blue"])
    _, o_area = largest_contour_in_mask(orange_mask, MIN_AREA["orange"])
    # threshold to reduce false positives
    if b_area >= MIN_AREA["blue"] and b_area > o_area:
        return "blue"
    if o_area >= MIN_AREA["orange"] and o_area > b_area:
        return "orange"
    return None

# ======================================================
# ACTIONS: maneuvers
# ======================================================
def do_corner_turn(color):
    """Time-based minor corner turn: NEUTRAL +/- CORNER_TURN_OFFSET for CORNER_TURN_TIME"""
    print(f"[TURN] corner turn: {color}")
    if color == "blue":
        target = STEERING_NEUTRAL - CORNER_TURN_OFFSET  # left
    else:
        target = STEERING_NEUTRAL + CORNER_TURN_OFFSET  # right

    set_servo_duty(target)
    drive_forward(MOTOR_SPEED)
    t0 = now()
    while now() - t0 < CORNER_TURN_TIME:
        # constantly check immediate collision
        d = read_all_distances()
        if any((d[s] is not None and d[s] <= COLLISION_THRESHOLD) for s in d):
            print("[TURN] collision during corner; invoking avoidance")
            do_collision_avoidance(d)
            # after avoidance we let the loop pick up next behavior
            return
        sleep(0.01)
    # stabilize
    set_servo_duty(STEERING_NEUTRAL)
    drive_forward(MOTOR_SPEED)
    sleep(CORNER_STABILIZE)

def do_block_pass(color):
    """Time-based pass for red/green:
       - red -> pass on right: strong right then slight left correction
       - green -> pass on left: strong left then slight right correction
       During pass only collision avoidance is active.
    """
    print(f"[PASS] passing block color={color}")
    if color == "red":
        # pass on right
        strong = STEERING_NEUTRAL + PASS_STRONG_OFFSET
        correction = STEERING_NEUTRAL - PASS_CORRECTION_OFFSET
    else:
        # green -> pass on left
        strong = STEERING_NEUTRAL - PASS_STRONG_OFFSET
        correction = STEERING_NEUTRAL + PASS_CORRECTION_OFFSET

    # strong turn
    set_servo_duty(strong)
    drive_forward(REDUCED_PWM)
    t0 = now()
    while now() - t0 < PASS_STRONG_TIME:
        d = read_all_distances()
        if any((d[s] is not None and d[s] <= COLLISION_THRESHOLD) for s in d):
            print("[PASS] collision during pass -> aborting to avoidance")
            do_collision_avoidance(d)
            return
        sleep(0.01)

    # correction
    set_servo_duty(correction)
    drive_forward(REDUCED_PWM)
    t1 = now()
    while now() - t1 < PASS_CORRECTION_TIME:
        d = read_all_distances()
        if any((d[s] is not None and d[s] <= COLLISION_THRESHOLD) for s in d):
            print("[PASS] collision during correction -> aborting to avoidance")
            do_collision_avoidance(d)
            return
        sleep(0.01)

    # straighten and resume forward
    set_servo_duty(STEERING_NEUTRAL)
    drive_forward(MOTOR_SPEED)

# ======================================================
# COLLISION AVOIDANCE (cases 1..6)
# ======================================================
def do_collision_avoidance(d):
    """Perform avoidance based on which sensors are triggered.
       d is dict {'left':cm,'front':cm,'right':cm} (values may be None)
    """
    L = d["left"]
    C = d["front"]
    R = d["right"]

    def below(x):
        return (x is not None) and (x <= COLLISION_THRESHOLD)

    triggered = (below(L), below(C), below(R))
    print(f"[AVOID] triggered={triggered} distances L={L} C={C} R={R}")

    # Stop immediately
    stop_drive()
    sleep(0.02)

    # Reverse (time-based short reverse)
    drive_reverse(REVERSE_DUTY)
    sleep(REVERSE_TIME_S)
    stop_drive()
    sleep(0.02)

    # Decide steer-away and forward action
    # Case priority: (L, C, R)
    if below(L) and not below(C) and not below(R):
        # 1) Left only -> reverse + steer right
        print("[AVOID] Left only -> steer right")
        set_servo_duty(STEERING_NEUTRAL + 1.0)
        drive_forward(REDUCED_PWM)
        sleep(0.25)
    elif below(R) and not below(C) and not below(L):
        # 2) Right only -> reverse + steer left
        print("[AVOID] Right only -> steer left")
        set_servo_duty(STEERING_NEUTRAL - 1.0)
        drive_forward(REDUCED_PWM)
        sleep(0.25)
    elif below(C) and not below(L) and not below(R):
        # 3) Center only -> reverse then choose side with more clearance
        print("[AVOID] Front only -> reverse and choose side")
        # read updated distances
        d2 = read_all_distances()
        left_clear = d2["left"] if d2["left"] is not None else 1000
        right_clear = d2["right"] if d2["right"] is not None else 1000
        if left_clear > right_clear:
            set_servo_duty(STEERING_NEUTRAL - 1.0)  # steer left
        else:
            set_servo_duty(STEERING_NEUTRAL + 1.0)  # steer right
        drive_forward(REDUCED_PWM)
        sleep(0.30)
    elif below(L) and below(C) and not below(R):
        # 4) Left+Center -> reverse + steer right
        print("[AVOID] Left+Center -> steer right")
        set_servo_duty(STEERING_NEUTRAL + 1.0)
        drive_forward(REDUCED_PWM)
        sleep(0.30)
    elif below(R) and below(C) and not below(L):
        # 5) Right+Center -> reverse + steer left
        print("[AVOID] Right+Center -> steer left")
        set_servo_duty(STEERING_NEUTRAL - 1.0)
        drive_forward(REDUCED_PWM)
        sleep(0.30)
    else:
        # 6) All three or ambiguous -> reverse longer and steer left as default
        print("[AVOID] All sensors -> reverse longer & steer left default")
        drive_reverse(REVERSE_DUTY)
        sleep(REVERSE_TIME_S * 2)
        stop_drive()
        set_servo_duty(STEERING_NEUTRAL - 1.0)
        drive_forward(REDUCED_PWM)
        sleep(0.35)

    # straighten and resume normal forward
    set_servo_duty(STEERING_NEUTRAL)
    drive_forward(MOTOR_SPEED)
    print("[AVOID] completed")

# ======================================================
# CENTER ALIGN small nudge
# ======================================================
def do_center_nudge(d):
    L = d["left"]; R = d["right"]
    if L is None or R is None:
        return False
    # only operate when in band (15..30 cm)
    if not ((COLLISION_THRESHOLD <= L < ALIGN_THRESHOLD) or (COLLISION_THRESHOLD <= R < ALIGN_THRESHOLD)):
        return False

    # small deadband
    if abs(L - R) < 2.0:
        return False

    if L > R:
        # more distance on left -> steer left (increase duty)
        target = STEERING_NEUTRAL + STEERING_STEP
    else:
        # steer right
        target = STEERING_NEUTRAL - STEERING_STEP

    set_servo_duty(target)
    drive_forward(REDUCED_PWM)
    t0 = now()
    while now() - t0 < 0.05:
        # safety check
        d2 = read_all_distances()
        if any((d2[s] is not None and d2[s] <= COLLISION_THRESHOLD) for s in d2):
            do_collision_avoidance(d2)
            return True
        sleep(0.005)

    # straighten
    set_servo_duty(STEERING_NEUTRAL)
    drive_forward(MOTOR_SPEED)
    return True

# ======================================================
# MAIN LOOP
# ======================================================
try:
    print("Starting main loop. Press Ctrl+C to stop.")
    # start moving immediately
    steer_neutral()
    drive_forward(MOTOR_SPEED)

    while True:
        frame = picam2.capture_array()
        # rotate 180 degrees for your camera orientation
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        H, W = frame.shape[:2]

        # read sensors first
        distances = read_all_distances()
        # distances dict: {'left':cm, 'front':cm, 'right':cm}

        # 1) Collision avoidance highest priority
        if any((distances[s] is not None and distances[s] <= COLLISION_THRESHOLD) for s in distances):
            do_collision_avoidance(distances)
            # show frame, then continue
            cv2.imshow("Robot View", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        # 2) find largest relevant contour across colors
        best = detect_largest_block(frame)  # returns (color, area, bbox) or (None,0,None)
        best_color, best_area, best_bbox = best

        # draw debug overlays
        cv2.putText(frame, f"L:{distances['left']:.1f}cm C:{distances['front']:.1f}cm R:{distances['right']:.1f}cm",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,0), 2)

        # If contour present display it
        if best_color is not None:
            x,y,wb,hb = best_bbox
            cx = x + wb // 2
            cy = y + hb // 2
            color_draw = (0,255,0) if best_color == "green" else (0,0,255) if best_color == "red" else (255,0,0) if best_color=="blue" else (0,165,255)
            cv2.rectangle(frame, (x,y), (x+wb, y+hb), color_draw, 2)
            cv2.putText(frame, f"{best_color} area={int(best_area)}", (x, max(10, y-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_draw, 2)

        # 3) Corner detection (immediate turn) - if not blocked and visible
        corner = detect_corner_line(frame)
        if corner is not None:
            # Do minor time-based corner turn then continue to normal stack
            do_corner_turn(corner)
            # after turn, immediately continue loop to check obstacle / alignment
            continue

        # 4) Block passing (takes precedence over lane alignment/center align)
        if best_color in ("red", "green") and best_area >= BLOCK_PASS_AREA_THRESHOLD:
            # ensure block is aligned to required side before passing
            cx = best_bbox[0] + best_bbox[2]//2
            if (best_color == "red" and cx < W * 0.5) or (best_color == "green" and cx > W * 0.5):
                # aligned to the required side (red->left, green->right)
                do_block_pass(best_color)
                # after pass, continue loop (next block will appear)
                continue

        # 5) Lane alignment (if block visible)
        if best_color is not None:
            cx = best_bbox[0] + best_bbox[2]//2
            # For red: we want block on left side -> if not enough left, steer left, etc.
            if best_color == "red":
                if cx > W * 0.5:
                    # block too right, steer left a bit
                    set_servo_duty(STEERING_NEUTRAL - 0.4)
                    drive_forward(REDUCED_PWM)
                    sleep(0.12)
                    steer_neutral()
                    drive_forward(MOTOR_SPEED)
            elif best_color == "green":
                if cx < W * 0.5:
                    # block too left, steer right a bit
                    set_servo_duty(STEERING_NEUTRAL + 0.4)
                    drive_forward(REDUCED_PWM)
                    sleep(0.12)
                    steer_neutral()
                    drive_forward(MOTOR_SPEED)
            else:
                # for blue/orange as a block case we don't use lane alignment
                pass

        else:
            # 6) If no block visible: run center nudges if in band
            do_center_nudge(distances)

        # show camera feed
        cv2.imshow("Robot View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("User interrupt — stopping")

finally:
    print("Cleanup: stopping motors & camera")
    try:
        stop_drive()
        steer_neutral()
    except Exception:
        pass
    pwm.stop()
    servo.stop()
    picam2.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()
