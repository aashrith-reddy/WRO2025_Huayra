# Raspberry Pi Dependencies Installation Guide

This document describes how to set up all required system dependencies, Python libraries, and interfaces on a Raspberry Pi 5 (or any Raspberry Pi running Raspberry Pi OS) using the terminal.

---

**1. Update System**
```bash
  sudo apt update
  sudo apt upgrade -y
```
**2. Install System Dependencies via apt**
```bash
    sudo apt install -y \
    python3-opencv \
    python3-pip \
    python3-picamera2 \
    python3-numpy \
    python3-smbus \
    python3-pigpio \
    python3-rpi.gpio \
```
**3. Creating a Virtual Environment and installing all required packages via pip**
    
```bash
    python3 -m venv venv
    source venv/bin/activate
```
```bash
    pip3 install \
    adafruit-circuitpython-vl53l0x \
    RPi.GPIO \
    pigpio \
    numpy \
    opencv-python \
    opencv-contrib-python \
    picamera2
```
When you are finished working within the virtual environment, you can deactivate it and return to the global Python environment by typing:
```bash
    deactivate
```

**4. Enabling Camera Interface**

   Enable hardware interfaces using raspi-config:
```bash
  sudo raspi-config
```
Navigate through: Interface Options → Enable Camera

**5. Start pigpio Daemon****

    The pigpio daemon must be running for motor control and steering systems:
```bash
    sudo systemctl enable pigpiod
    sudo systemctl start pigpiod
```

**6. Verification**

   To check if installations succeeded:
   ```bash
    python3 -c "import cv2, picamera2, smbus2, RPi.GPIO, pigpio; print('All dependencies installed successfully!')"
   ```

---

**Reboot your Raspberry Pi after enabling these options by running the following:**
```bash
sudo reboot
```

---

### (OPTIONAL) Adjusting the Raspberry Pi 5 Fan's Minimum Temperature

To adjust the minimum temperature at which your Raspberry Pi 5's fan starts, you need to edit the **`config.txt`** file. This file contains various configuration settings for your Pi. You should **not** attempt to modify `.dtb` files, as they are compiled and not meant for manual editing.

---

**Step 1: Accessing the `config.txt` File**

  The configuration file is located at `/boot/firmware/config.txt`. You can edit it using a text editor like `nano`. Open a terminal and run the following command to begin editing:

```bash
  sudo nano /boot/firmware/config.txt
```
**Step 2: Modifying the Fan Settings**

  Once the file is open, you can add or modify the parameters related to the fan's behavior. These parameters are documented in the README file located at /boot/firmware/overlays/README, but remember not to edit the README file itself.
The key parameters for the fan's first cooling level are:
  1. fan_temp0: The temperature threshold (in millicelsius) for the fan to turn on. The default is 50,000 mC (50°C).
  2. fan_temp0_hyst: The temperature hysteresis (in millicelsius). The fan will turn off when the temperature drops fan_temp0_hyst below fan_temp0. The default is 5,000 mC (5°C).
  3. fan_temp0_speed: The fan's PWM speed setting, a value between 0 and 255. The default is 75.

To change the fan's minimum operating temperature, you should add the following lines to the end of your config.txt file. These settings will start the fan at 20°C and turn it off at 10°C.

```Bash
  dtparam=fan_temp0=20000
  dtparam=fan_temp0_hyst=10000
  dtparam=fan_temp0_speed=125
```

**Step 3: Saving and Exiting**

  After adding the lines, save the file by pressing Ctrl + X, then Y to confirm, and finally Enter. The changes will take effect after you reboot your Raspberry Pi.
