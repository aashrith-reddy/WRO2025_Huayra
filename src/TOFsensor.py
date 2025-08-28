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
