import time

float left = 0
float center = 0
float right = 0

# Assume these functions get distances in cm from the sensors
def get_left_distance():
    global left
    # Replace with actual sensor reading code
    return round(left,2) 

def get_center_distance():
    global center
    return round(center,2)

def get_right_distance():
    global right
    return round(right,2)

# Threshold for minimum safe distance in reverse (in cm)
# Will test accurately later when sensors are mounted
DANGER_THRESHOLD = 50

def check_for_obstacle():
    left = get_left_distance()
    center = get_center_distance()
    right = get_right_distance()

    print(f"Left: {left} cm | Center: {center} cm | Right: {right} cm")

    if left < DANGER_THRESHOLD or center < DANGER_THRESHOLD or right < DANGER_THRESHOLD:
        return True  # Obstacle detected
    return False

def reverse_motion():
    print("🔄 Engaging Reverse Mode...")
    while True:
        if check_for_obstacle():
            print("Obstacle detected! Stopping reverse motion.")
            stop_vehicle()
            break
        else:
            move_backward()
        time.sleep(0.1)  # Sensor check interval

def move_backward():
    # Replace with your motor control code
    print("Moving backward...")

def stop_vehicle():
    # Replace with motor stop code
    print("Motors stopped.")