import pigpio

STEER_GPIO = 18
STEER_CENTER_US = 1500
STEER_RANGE_US = 500
STEER_MAX_ANGLE = 30

pi = pigpio.pi()
pi.set_mode(STEER_GPIO, pigpio.OUTPUT)
pi.set_PWM_frequency(STEER_GPIO, 50)

def angle_to_pulse(angle):
    angle = max(min(angle, STEER_MAX_ANGLE), -STEER_MAX_ANGLE)
    return STEER_CENTER_US + int((angle / STEER_MAX_ANGLE) * STEER_RANGE_US)

def set_steering(angle):
    pi.set_servo_pulsewidth(STEER_GPIO, angle_to_pulse(angle))
