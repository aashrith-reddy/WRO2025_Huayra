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
