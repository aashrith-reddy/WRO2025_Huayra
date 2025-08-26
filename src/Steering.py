from gpiozero import Servo
from time import sleep

servo = Servo(17, min_pulse_width=0.0005, max_pulse_width=0.0025)

try:
    while True:
        servo.value = -0.67
        sleep(1)
        servo.mid()
        sleep(1)
        servo.value = 0.67
        sleep(1)
except KeyboardInterrupt:
    servo.mid()
