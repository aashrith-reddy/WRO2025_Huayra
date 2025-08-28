# Software Documentation

This repository contains **all modular scripts** used to build the final code. Below is a **script-by-script feature list** for the entire project:

---

### camera.py
```bash
from picamera2 import Picamera2
import cv2, time

cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"format":"RGB888","size":(640,480)}))
cam.start(); time.sleep(0.5)

while True:
    f = cam.capture_array()
    cv2.imshow("Feed", f)
    if cv2.waitKey(1)&0xFF==ord('q'): break

cam.stop(); cv2.destroyAllWindows()
```

**1.	Imports required libraries** \
&nbsp;&nbsp;&nbsp;•	picamera2 to control Raspberry Pi Camera \
&nbsp;&nbsp;&nbsp;•	cv2 (OpenCV) to show frames in a window \
&nbsp;&nbsp;&nbsp;•	time for adding a small delay after starting the camera
```bash
from picamera2 import Picamera2
import cv2, time
```

**2.	Initializes the camera** \
&nbsp;&nbsp;&nbsp;•	Creates a Picamera2 object under the variable name cam. \
&nbsp;&nbsp;&nbsp;•	Configures it to output **640×480 frames** in **RGB888 format**.
```bash
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"format":"RGB888","size":(640,480)}))
```
**3.	Starts the camera** \
&nbsp;&nbsp;&nbsp;•	Starts streaming frames. \
&nbsp;&nbsp;&nbsp;•	Waits 0.5 seconds to allow the sensor to stabilize.
```bash
cam.start(); time.sleep(0.5)
```
**4.	Main loop to display video** \
&nbsp;&nbsp;&nbsp;•	Displays them in a window titled “Feed”. \
&nbsp;&nbsp;&nbsp;•	Exits when the ‘q’ key is pressed.
```bash
while True:
    f = cam.capture_array()
    cv2.imshow("Feed", f)
    if cv2.waitKey(1)&0xFF==ord('q'): break
```
**5. Stops the camera** \
&nbsp;&nbsp;&nbsp;•	Stops the camera stream. \
&nbsp;&nbsp;&nbsp;•	Closes any OpenCV windows.
```bash
cam.stop(); cv2.destroyAllWindows()
```

---

### **TOFsensor.py**
```bash
import time
import board
import busio
from gpiozero import LED
from adafruit_vl53l0x import VL53L0X

xshut_pin = LED(17)
i2c = busio.I2C(board.SCL, board.SDA)
xshut_pin.off()
time.sleep(0.1)
xshut_pin.on()
time.sleep(0.1)
sensor = VL53L0X(i2c)

try:
    while True:
        distance = sensor.range /10
        print(f"Distance: {distance} cm")
        time.sleep(1)
except KeyboardInterrupt:
    print("Program stopped")
```

**1. Imports required libraries** \
&nbsp;&nbsp;&nbsp;•	time — used for adding short delays. \
&nbsp;&nbsp;&nbsp;•	board — maps Raspberry Pi pins (SCL, SDA, etc.) for I²C communication. \
&nbsp;&nbsp;&nbsp;•	busio.I2C — handles I²C protocol communication. \
&nbsp;&nbsp;&nbsp;•	gpiozero.LED — convenient way to control a GPIO pin as a simple on/off output (here, it’s used for XSHUT rather than an actual LED). \
&nbsp;&nbsp;&nbsp;•	adafruit_vl53l0x.VL53L0X — library to talk to the VL53L0X time-of-flight distance sensor.
```bash
import time
import board
import busio
from gpiozero import LED
from adafruit_vl53l0x import VL53L0X
```

**2. Hardware Setup** \
&nbsp;&nbsp;&nbsp;•	xshut_pin is GPIO17 configured as an output using gpiozero’s LED class. \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;•	Even though it’s called LED, it’s just a digital output that can be turned on/off. \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;•	This pin is connected to the XSHUT (shutdown) pin of the VL53L0X sensor. \
&nbsp;&nbsp;&nbsp;•	i2c = busio.I2C(board.SCL, board.SDA) sets up the I²C bus using the Pi’s hardware SCL and SDA pins.
```bash
xshut_pin = LED(17)
i2c = busio.I2C(board.SCL, board.SDA)
```

**3. Reset the sensor using XSHUT** \
&nbsp;&nbsp;&nbsp;•	xshut_pin.off() drives GPIO17 low due to which VL53L0X is held in reset (powered off). \
&nbsp;&nbsp;&nbsp;•	Wait 0.1 s for it to fully power down. \
&nbsp;&nbsp;&nbsp;•	xshut_pin.on() drives GPIO17 high due to which VL53L0X powers up again. \
&nbsp;&nbsp;&nbsp;•	Wait another 0.1 s for the sensor to initialize. \ \

&nbsp;&nbsp;&nbsp;This reset sequence ensures the sensor is in a clean state, and is especially useful when you have multiple sensors on the same I²C bus &nbsp;&nbsp;&nbsp;(you can power them up one at a time and give them unique addresses).
```bash
xshut_pin.off()
time.sleep(0.1)
xshut_pin.on()
time.sleep(0.1)
```

**4. Create the sensor object** \
&nbsp;&nbsp;&nbsp;•	Initializes the Adafruit VL53L0X driver on the I²C bus. \
&nbsp;&nbsp;&nbsp;•	The library automatically uses the default address 0x29 unless you change it.
```bash
sensor = VL53L0X(i2c)
```

**5. Main Loop** \
&nbsp;&nbsp;&nbsp;•	sensor.range returns distance in millimeters (mm). \
&nbsp;&nbsp;&nbsp;•	Dividing by 10 converts to centimeters (cm). \
&nbsp;&nbsp;&nbsp;•	Prints out the distance every 1 second.
```bash
try:
    while True:
        distance = sensor.range / 10
        print(f"Distance: {distance} cm")
        time.sleep(1)
```

**6. Exit** \
&nbsp;&nbsp;&nbsp;•	If you press Ctrl + C, the loop ends and prints a message.
```bash
except KeyboardInterrupt:
    print("Program stopped")
```

---

### **Steering.py**
```bash
from rpi_hardware_pwm import HardwarePWM
from time import sleep

pwm = HardwarePWM(pwm_channel=0, hz=50, chip=0)
pwm.start(5)

try:
    while True:
        pwm.change_duty_cycle(5)
        sleep(2)
        pwm.change_duty_cycle(7.5)
        sleep(2)
        pwm.change_duty_cycle(9.7)
        sleep(2)
        pwm.change_duty_cycle(7.5)
        sleep(2)

except KeyboardInterrupt:
    pwm.stop()
```
**1. Imports Required Libraries** \
&nbsp;&nbsp;&nbsp;•	rpi_hardware_pwm.HardwarePWM lets you control the Pi’s built-in hardware PWM channels. Hardware PWM is more stable and jitter-&nbsp;&nbsp;&nbsp;free than software PWM (like in RPi.GPIO), which is especially important for servos. \
&nbsp;&nbsp;&nbsp;•	sleep() pauses the program for a specified time in seconds.
```bash
from rpi_hardware_pwm import HardwarePWM
from time import sleep
```
**2. Create PWM object** \
&nbsp;&nbsp;&nbsp;•	pwm_channel=0 — uses hardware PWM channel 0 (the Pi has two: 0 and 1). \
&nbsp;&nbsp;&nbsp;•	hz=50 — sets frequency to 50 Hz (period = 20 ms), standard for servo motors. \
&nbsp;&nbsp;&nbsp;•	chip=0 — refers to /sys/class/pwm/pwmchip0, the first hardware PWM controller on the Pi.
```bash
pwm = HardwarePWM(pwm_channel=0, hz=50, chip=0)
```
**3. Start PWM with 5% duty cycle** \
&nbsp;&nbsp;&nbsp;•  This enables PWM output on that channel with a duty cycle of 5% (i.e., the signal is HIGH for 1 ms and LOW for 19 ms, since 20 ms &nbsp;&nbsp;&nbsp;period × 5% = 1 ms). \
&nbsp;&nbsp;&nbsp;•	For a hobby servo: \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;•	~5% → far left position (~0°) \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;•	~7.5% → center (~90°) \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;•	~10% → far right (~180°)
```bash
pwm.start(5)
```
**4. Main Loop** \
&nbsp;&nbsp;&nbsp;•	5% → servo moves to left position \
&nbsp;&nbsp;&nbsp;•	7.5% → servo centers \
&nbsp;&nbsp;&nbsp;•	9.7% → servo moves to right position \
&nbsp;&nbsp;&nbsp;•	7.5% → servo centers again
```bash
try:
    while True:
        pwm.change_duty_cycle(5)
        sleep(2)
        pwm.change_duty_cycle(7.5)
        sleep(2)
        pwm.change_duty_cycle(9.7)
        sleep(2)
        pwm.change_duty_cycle(7.5)
        sleep(2)
```

**5. Exit**
&nbsp;&nbsp;&nbsp;•	When you press Ctrl + C, the program exits the loop.
&nbsp;&nbsp;&nbsp;•	pwm.stop() disables PWM output so the pin no longer outputs a PWM signal.
```bash
except KeyboardInterrupt:
    pwm.stop()
```

---

### **MotorDriver.py**
```bash
from gpiozero import DigitalOutputDevice
from rpi_hardware_pwm import HardwarePWM
from time import sleep

direction = DigitalOutputDevice(17)
pwm = HardwarePWM(pwm_channel=1, hz=18000, chip=0)

try:
    direction.value = 1
    pwm.start(30)
    print("Motor running forward at 30%...")
    sleep(5)

    pwm.stop()
    print("Motor stopped.")
    sleep(1)

    direction.value = 0
    pwm.start(30)
    print("Motor running backward at 30%...")
    sleep(5)

    pwm.stop()
    print("Motor stopped.")

except KeyboardInterrupt:
    print("Interrupted by user.")
    pwm.stop()
```
**1. Imports required Libraries** \
&nbsp;&nbsp;&nbsp;•	DigitalOutputDevice (gpiozero) — lets you easily set a GPIO pin HIGH or LOW to control directio \
&nbsp;&nbsp;&nbsp;•	HardwarePWM (rpi_hardware_pwm) — generates a very stable PWM signal directly using the Pi’s hardware \
&nbsp;&nbsp;&nbsp;•	sleep: Used to pause execution for a given number of seconds.
```bash
from gpiozero import DigitalOutputDevice
from rpi_hardware_pwm import HardwarePWM
from time import sleep
```
**2. Hardware setup** \
&nbsp;&nbsp;&nbsp;•	direction → GPIO17 controls the motor’s rotation direction. \
&nbsp;&nbsp;&nbsp;•	pwm → hardware PWM on channel 1 at 18 kHz frequency. \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;•	18 kHz is above human hearing which means that no audible whining from the motor would be heard.
```bash
direction = DigitalOutputDevice(17)
pwm = HardwarePWM(pwm_channel=1, hz=18000, chip=0)
```
**3. Main Loop** \
&nbsp;&nbsp;&nbsp;•	direction.value: \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;• =1 → sets GPIO17 HIGH which makes the motor driver rotate the motor forward. \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;• =0 → sets GPIO17 LOW which makes the motor driver rotate the motor backward. \
&nbsp;&nbsp;&nbsp;•	pwm.start(30) → starts PWM with 30% duty cycle, meaning the motor gets ~30% of its speed. \
&nbsp;&nbsp;&nbsp;•	sleep() → waits for an amount of time
```bash
try:
    direction.value = 1
    pwm.start(30)
    print("Motor running forward at 30%...")
    sleep(5)

    pwm.stop()
    print("Motor stopped.")
    sleep(1)

    direction.value = 0
    pwm.start(30)
    print("Motor running backward at 30%...")
    sleep(5)

    pwm.stop()
    print("Motor stopped.")
```
**4. Exit**
&nbsp;&nbsp;&nbsp;•	If you press Ctrl + C, the program exits and ensures PWM is stopped so the motor doesn’t keep running unexpectedly.
```bash
except KeyboardInterrupt:
    print("Interrupted by user.")
    pwm.stop()
```
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


### **ColorDetection.py**
**Features:**
- **Lane Detection**: Finds offset from center between orange and blue lines.
- **Pillar Detection**: Identifies red and green pillars (two HSV ranges for red).
- **Parking Marker Detection**: Finds magenta region center.

---
