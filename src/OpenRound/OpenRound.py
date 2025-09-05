from time import sleep
from picamera2 import Picamera2
import serial
import cv2
import serial
import time
import numpy as np

laps=0

arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

picam2 = Picamera2()
picam2.preview_configuration.size = (640, 480)
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

blowerLimit, bupperLimit = (np.array([95, 80, 60]), np.array([125, 255, 255]))
olowerLimit, oupperLimit = (np.array([5, 120, 120]), np.array([22, 255, 255]))
mlowerLimit, mupperLimit = (np.array([140, 80, 60]), np.array([165, 255, 255]))
glowerLimit, gupperLimit = (np.array([40, 80, 60]), np.array([85, 255, 255]))
rlowerLimit1, rupperLimit1 = (np.array([0, 80, 60]), np.array([10, 255, 255]))
rlowerLimit2, rupperLimit2 = (np.array([170, 80, 60]), np.array([180, 255, 255]))

try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        bmask = cv2.inRange(hsvImage, blowerLimit, bupperLimit)
        omask = cv2.inRange(hsvImage, olowerLimit, oupperLimit)
        gmask = cv2.inRange(hsvImage, glowerLimit, gupperLimit)
        mmask = cv2.inRange(hsvImage, mlowerLimit, mupperLimit)
        rmask1 = cv2.inRange(hsvImage, rlowerLimit1, rupperLimit1)
        rmask2 = cv2.inRange(hsvImage, rlowerLimit2, rupperLimit2)
        rmask = cv2.bitwise_or(rmask1, rmask2)

        b_conn_comp = cv2.connectedComponentsWithStats(bmask)
        o_conn_comp = cv2.connectedComponentsWithStats(omask)
        m_conn_comp = cv2.connectedComponentsWithStats(mmask)
        g_conn_comp = cv2.connectedComponentsWithStats(gmask)
        r_conn_comp = cv2.connectedComponentsWithStats(rmask)

        b_signal = [i[4] for i in b_conn_comp[2][1:] if i[4] > 8000]
        o_signal = [i[4] for i in o_conn_comp[2][1:] if i[4] > 8000]
        m_signal = [i[4] for i in m_conn_comp[2][1:] if i[4] > 8000]
        g_signal = [i[4] for i in g_conn_comp[2][1:] if i[4] > 8000]
        r_signal = [i[4] for i in r_conn_comp[2][1:] if i[4] > 8000]

        if laps == 12:
            sleep(200)
            break
        elif b_signal:
            print("Blue")
            sleep(400)
            laps += 1
        elif o_signal:
            print("Orange")
            sleep(400)
            laps += 1

        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    ser.close()
