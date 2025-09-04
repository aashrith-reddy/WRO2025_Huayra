import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)
print("Raspberry Pi ready")

arduino.write(b'M')
time.sleep(1)

while True:
    if arduino.in_waiting > 0:
        line = arduino.readline().decode('utf-8').rstrip()
        print("Arduino says:", line)
        time.sleep(0.1)
