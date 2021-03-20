import cv2


class PiRGBArray:
    def __init__(self, cam, size):
        self.size = size
        self.array = None


class PiCamera:
    def __init__(self):
        self.resolution = None
        self.framerate = None
        self.videoCapture = cv2.VideoCapture(0)

        # wait until the camera returns something
        print("INITIALIZING CAMERA...")
        while not self.videoCapture.read()[0]:
            pass
        print("...CAMERA INITIALIZED")

    def capture(self, array, format, use_video_port):
        # assume use_video_port=true and format="bgr"
        ret_val, img = self.videoCapture.read()
        if not ret_val: raise ConnectionError("camera error")
        array.array = cv2.resize(img, array.size)

    def close(self):
        self.videoCapture.release()
