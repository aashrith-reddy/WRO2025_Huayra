# Software Doccumentation

This repository contains **all modular scripts** used to build the final code. Below is a **script-by-script feature list** for the entire project:

---

### **Camera.py**
**Features:**
- Captures live frames from the Raspberry Pi Camera v1.3.
- Uses the PiCamera2 API for stable frame rates and OpenCV compatibility.
- Converts camera output from RGB888 to BGR format for OpenCV processing.

---

### **TOF.py**
**Features:**
- Handles **three VL53L0X Time-of-Flight sensors** via I²C.
- Uses **XSHUT pins** to power each sensor individually and assign **unique I²C addresses** (`0x29`, `0x2A`, `0x2B`).
- Reads distances in **millimeters** with high precision.
- Provides a **multi-sensor initialization** function for synchronized startup.

**Key Formula:**  
The VL53L0X sensor internally measures time-of-flight and converts to distance via: distance_mm = (time_of_flight_seconds × speed_of_light_mm_per_second) ÷ 2
(The division by 2 accounts for the round-trip time.)

---

### **Steering.py**
**Features:**
- Converts desired **steering angle** (°) into **servo PWM pulse width**.
- Pulse width is calculated with linear scaling between `-30°` and `+30°`.
- Protects against mechanical oversteer by clamping angles.

**Formula:** pulse_width_us = center_pulse_us + (angle_deg ÷ max_angle_deg) × range_us

---

### **StartButton.py**
**Features:**
- Waits for **Power Switch** to be ON before running.
- Waits for **Start Button** press to begin execution.
- Prevents premature robot movement.

---

### **PID.py**
**Features:**
- Implements a **PID controller** for steering and/or speed control.
- Adjustable **Kp, Ki, Kd** constants.
- **Integrator clamping** to avoid wind-up.

**Formula:** output = (Kp × error) + (Ki × ∑error × Δt) + (Kd × (error - prev_error) ÷ Δt)

Where:
- `error` = target_value − current_value
- `Δt` = time difference between updates

---

### **MotorDriver.py**
**Features:**
- Controls **rear DC motor** via PWM and direction pins.
- Accepts speed in range `-1.0` (full reverse) to `+1.0` (full forward).
- Converts speed value to **PWM duty cycle**.

**Formula:**
pwm_value = abs(speed) × 255 \
direction = HIGH if speed ≥ 0 else LOW

---

### **LineTracking_LaneEstimation.py**
**Features:**
- Detects **orange** and **blue** lane lines using HSV color thresholds.
- Finds **largest contour** for each color.
- Calculates **thickness in pixels** for distance estimation.
- Computes **lane center offset** in pixels and millimeters.
- Identifies **which color is closer** to the camera.

**Key Formulas:**
1. Distance to Line (Using Focal Length):
   distance_mm = (real_width_mm × focal_length_px) ÷ perceived_width_px
2. Lateral Offset (Pixels to mm):
   lateral_offset_mm = (offset_px × distance_mm) ÷ focal_length_px
3. Focal Length Calibration:
   focal_length_px = (perceived_width_px × known_distance_mm) ÷ real_width_mm
---

### **HSVcalibration.py**
**Features:**
- Interactive trackbars for **Hue**, **Saturation**, **Value** tuning.
- Displays real-time mask and detection output.
- Prints calibrated lower/upper HSV bounds for use in detection scripts.

---

### **ColorDetection.py**
**Features:**
- **Lane Detection**: Finds offset from center between orange and blue lines.
- **Pillar Detection**: Identifies red and green pillars (two HSV ranges for red).
- **Parking Marker Detection**: Finds magenta region center.

---

# Installation

### Update package list
sudo apt update \
sudo apt upgrade -y

### Install system dependencies
sudo apt install -y \
    python3-opencv \
    python3-pip \
    python3-picamera2 \
    python3-numpy \
    python3-smbus \
    python3-pigpio \
    python3-rpi.gpio \
    libatlas-base-dev \
    libopenjp2-7 \
    libtiff5

### Install Python packages from pip
pip3 install \
    adafruit-circuitpython-vl53l0x \
    RPi.GPIO \
    pigpio \
    numpy \
    smbus2 \
    opencv-python \
    opencv-contrib-python \
    picamera2

### Enable I2C and Camera Interfaces
sudo raspi-config \
→ Interface Options → Enable I2C \
→ Interface Options → Enable Camera

### Start pigpio daemon (needed for MotorDriver and Steering control)
sudo systemctl enable pigpiod \
sudo systemctl start pigpiod
