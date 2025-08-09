from picamera2 import Picamera2
import cv2, time

class Camera:
    def __init__(self, size=(640,480), fps=30):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration({"format":"RGB888", "size": size})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(0.2)
    def read(self):
        arr = self.picam2.capture_array()
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    def close(self):
        self.picam2.stop()
