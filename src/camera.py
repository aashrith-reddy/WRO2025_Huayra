from picamera2 import Picamera2
import cv2, time

cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"format":"RGB888","size":(640,480)}))
cam.start(); time.sleep(0.5)

while True:
    f = cam.capture_array()
    cv2.imshow("Feed", f)
    if cv2.waitKey(1)&0xFF==ord('q'): break

cam.stop(); cv2.destroyAllWindows()
