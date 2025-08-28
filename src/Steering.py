from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory
from time import sleep

factory = LGPIOFactory(chip=0)
servo=AngularServo(23)
try:
    while True:
        servo.angle=30
        sleep(1)
        servo.angle=-30
        sleep(1)
except KeyboardInterrupt:
    servo.close()
