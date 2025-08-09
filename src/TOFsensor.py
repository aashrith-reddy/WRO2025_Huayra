# TOFsensor.py
import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO

# === GPIO pins connected to the XSHUT pins of each VL53L0X ===
# Change these to match your wiring!
XSHUT_PINS = [5, 6, 13]  # Example: sensor 1, sensor 2, sensor 3

# === Unique addresses for each sensor ===
TOF_ADDRESSES = [0x29, 0x2A, 0x2B]

# === Setup GPIO for XSHUT control ===
GPIO.setmode(GPIO.BCM)
for pin in XSHUT_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)  # Keep all sensors off initially

# === Create I2C bus ===
i2c = busio.I2C(board.SCL, board.SDA)

class TOFSensor:
    def __init__(self, index):
        """
        index: 0, 1, or 2 for sensor number
        """
        if index < 0 or index >= len(XSHUT_PINS):
            raise ValueError("Invalid sensor index")

        self.index = index
        self.address = TOF_ADDRESSES[index]
        self.sensor = None

    def init_sensor(self):
        """
        Power up this sensor, assign a unique address.
        Call this at startup for all sensors one-by-one.
        """
        # Turn on only this sensor
        GPIO.output(XSHUT_PINS[self.index], GPIO.HIGH)
        time.sleep(0.05)  # Small delay for sensor boot

        # Create a temporary sensor object at default address
        temp_sensor = adafruit_vl53l0x.VL53L0X(i2c)
        # Change to unique address
        temp_sensor.set_address(self.address)

        # Now connect to it at its new address
        self.sensor = adafruit_vl53l0x.VL53L0X(i2c, address=self.address)

    def read_mm(self):
        """
        Return distance in mm.
        """
        if self.sensor is None:
            raise RuntimeError(f"Sensor {self.index} not initialized")
        return self.sensor.range

# === Function to initialize all sensors ===
def init_all_sensors():
    sensors = [TOFSensor(i) for i in range(len(XSHUT_PINS))]

    # Keep all sensors off
    for pin in XSHUT_PINS:
        GPIO.output(pin, GPIO.LOW)
    time.sleep(0.5)

    # Power up each sensor one at a time and assign addresses
    for s in sensors:
        # Turn off others
        for pin in XSHUT_PINS:
            GPIO.output(pin, GPIO.LOW)
        time.sleep(0.05)

        # Turn on this one and init
        s.init_sensor()

    return sensors

if __name__ == "__main__":
    try:
        all_sensors = init_all_sensors()
        while True:
            readings = [s.read_mm() for s in all_sensors]
            print("Distances (mm):", readings)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Stopping TOF test")
    finally:
        GPIO.cleanup()
