"""
Replacement of the picamera to use a webcam.
It implements the same API but uses the user webcam instead of the picamera
Use as:

try:
    import picamera
    from picamera.array import PiRGBArray
except:
    import classes.sim_picamera as picamera
    from classes.sim_picamera import PiRGBArray

"""
import cv2
import numpy as np


class PiRGBArray:
    def __init__(self, cam, size):
        self.size = size
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
        return img

    def capture(self, array, format, use_video_port):
        # assume use_video_port=true and format="bgr"
        array.array = cv2.resize(self._read_retry(), array.size)

    def close(self):
        self.videoCapture.release()
