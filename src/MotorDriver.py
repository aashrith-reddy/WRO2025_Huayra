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
