from gpiozero import DigitalOutputDevice
from rpi_hardware_pwm import HardwarePWM
from time import sleep

direction = DigitalOutputDevice(17)
pwm = HardwarePWM(pwm_channel=1, hz=18000, chip=0)

direction.value = 1
pwm.start(25)

try:
    while True:
        pass

except:
    pwm.stop()
