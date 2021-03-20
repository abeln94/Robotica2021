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
        self._read_retry()
        print("...CAMERA INITIALIZED")

    def _read_retry(self):
        ok = False
        img = None
        while not ok:
            ok, img = self.videoCapture.read()
        return img

    def capture(self, array, format, use_video_port):
        # assume use_video_port=true and format="bgr"
        array.array = cv2.resize(self._read_retry(), array.size)

    def close(self):
        self.videoCapture.release()
