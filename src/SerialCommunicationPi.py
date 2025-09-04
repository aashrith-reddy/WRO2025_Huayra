import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

def send_command(cmd):
    ser.write((cmd + '\n').encode())
    response = ser.readline().decode().strip()
    return response

while True:
    response = send_command("MAGENTA")
    print(response)
    time.sleep(2)
    
    response = send_command("RED")
    print(response)
    time.sleep(2)