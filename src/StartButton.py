import RPi.GPIO as GPIO, time

START_BUTTON_GPIO = 17
POWER_SWITCH_GPIO = 4

GPIO.setmode(GPIO.BCM)
GPIO.setup(START_BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(POWER_SWITCH_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def wait_for_start():
    while GPIO.input(POWER_SWITCH_GPIO) == 0:
        time.sleep(0.1)
    while GPIO.input(START_BUTTON_GPIO) == 0:
        time.sleep(0.05)
