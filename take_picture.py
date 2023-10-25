import cv2
import numpy as np
from threading import Thread


class Camera:
    def __init__(self, cam_num, directory):
        self.cap = cv2.VideoCapture(cam_num)
        self.cam_num = cam_num
        self.directory = directory

    def run(self):
        c = Thread(target=self.capture)
        c.start()
        while True:
            try:
                success, self.image = self.cap.read()
                if not success:
                    print("camera does not work")
                    return None

                self.image.flags.writeable = False
                cv2.imshow('camera {}'.format(self.cam_num), self.image)
                self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
                cv2.waitKey(1)
            except Exception as e:
                print(e)

    def capture(self):
        while True:
            i = input("capture: ")
            if i == 'c':
                cv2.imwrite(self.directory, self.image)
            elif i == 'd':
                cv2.destroyAllWindows()


if __name__ == "__main__":
    test_cam = Camera(0, "./images")
    test_cam.run()
