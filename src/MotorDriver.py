import pigpio, RPi.GPIO as GPIO

MOTOR_PWM_GPIO = 13
MOTOR_DIR1 = 22
MOTOR_DIR2 = 27

pi = pigpio.pi()
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_DIR1, GPIO.OUT)
GPIO.setup(MOTOR_DIR2, GPIO.OUT)
pi.set_PWM_frequency(MOTOR_PWM_GPIO, 2000)

def motor_set_speed(speed):
    s = max(min(speed, 1.0), -1.0)
    if s >= 0:
        GPIO.output(MOTOR_DIR1, GPIO.HIGH)
        GPIO.output(MOTOR_DIR2, GPIO.LOW)
    else:
        GPIO.output(MOTOR_DIR1, GPIO.LOW)
        GPIO.output(MOTOR_DIR2, GPIO.HIGH)
    pi.set_PWM_dutycycle(MOTOR_PWM_GPIO, int(abs(s) * 255))

def stop_motor():
    pi.set_PWM_dutycycle(MOTOR_PWM_GPIO, 0)
