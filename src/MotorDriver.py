from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep

dir_pin = DigitalOutputDevice(20)
pwm_pin = PWMOutputDevice(21, frequency=1000)

try:
    while True:
        dir_pin.on();  pwm_pin.value = 0.6; sleep(2)
        pwm_pin.value = 0;   sleep(1)
        dir_pin.off(); pwm_pin.value = 0.6; sleep(2) 
        pwm_pin.value = 0;   sleep(1)
except KeyboardInterrupt:
    pwm_pin.value = 0
