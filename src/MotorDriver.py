from gpiozero import PhaseEnableMotor
from time import sleep

motor = PhaseEnableMotor(phase=17, enable=18)

try:
    print("Moving forward at 30% speed...")
    motor.forward(0.3)
    sleep(2)

    print("Stopping motor...")
    motor.stop()
    sleep(2)
    
    print("Moving backward at 30% speed...")
    motor.backward(0.3)
    sleep(2)

    print("Stopping motor...")
    motor.stop()

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    motor.close()
