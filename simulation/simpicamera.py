"""
Replacement of the picamera to use a webcam.
It implements the same API but uses the user webcam instead of the picamera
Use as:

try:
    import picamera
    from picamera.array import PiRGBArray
except:
    import simpicamera as picamera
    from simpicamera import PiRGBArray

"""
import cv2
import numpy as np


class PiRGBArray:
    def __init__(self, cam, size):
        self.size = size
        self.array = None
    
    def truncate(self, value=0):
        # https://picamera.readthedocs.io/en/release-1.10/api_array.html#pirgbarray
        self.array = None


class PiCamera:
    def __init__(self):
        self.resolution = None
        self.framerate = None
        self.videoCapture = cv2.VideoCapture(0)

        if self.videoCapture.isOpened():
            # wait until the camera returns something
            print("INITIALIZING CAMERA...")
            self._read_retry()
            print("...CAMERA INITIALIZED")
        else:
            print("No camera connected, using the no-cam mode")

    def _read_retry(self):
        if not self.videoCapture.isOpened():
            # no-camera mode, a single black pixel
            return np.zeros((1, 1, 3), np.uint8)

        # retry until a frame is read
        ok = False
        img = None
        while not ok:
            ok, img = self.videoCapture.read()
        return img # cv2.flip(img, -1)  # to rotate 180

    def capture(self, array, format="bgr", use_video_port=True):
        assert use_video_port and format == "bgr"
        array.array = cv2.resize(self._read_retry(), array.size)

    def close(self):
        self.videoCapture.release()
