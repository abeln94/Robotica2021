"""
Simulation of a real robot, this replaces the BrickPi3 module.
It implements the same API which abstracts the real robot's engines, but functionality is simulated
Use as:

try:
    import brickpi3
except ModuleNotFoundError:
    import simbrickpi3 as brickpi3

"""
import random
from enum import Enum
from multiprocessing import Manager
from time import time

from classes.DeltaVal import SyncDeltaVal

FRICTION = 1  # 0.975
MIN_DISTANCE = 200  # 20 cm
MAX_DISTANCE = 2000  # 2 m


class BrickPi3:
    class SENSOR_TYPE(Enum):
        TOUCH = 1
        NXT_ULTRASONIC = 2

    PORT_1 = "PORT_1"
    PORT_A = "PORT_A"
    PORT_B = "PORT_B"
    PORT_C = "PORT_C"

    def __init__(self):
        m = Manager()
        self.encoders = m.dict()
        self.offsets = m.dict()
        self.dps = m.dict()
        self.lastUpdate = SyncDeltaVal()

    def reset_all(self):
        self.lastUpdate.reset()
        for d in [self.encoders, self.offsets, self.dps]:
            for m in d.keys():
                d[m] = 0

    def getOrDefault(self, dict, port):
        return dict[port] if port in dict else 0

    def set_sensor_type(self, port, type):
        pass

    def get_motor_encoder(self, port):
        self.update()
        return int(self.getOrDefault(self.encoders, port) - self.getOrDefault(self.offsets, port))

    def get_abs_motor_encoder(self, port):
        self.update()
        return self.getOrDefault(self.encoders, port)

    def offset_motor_encoder(self, port, value):
        self.update()
        self.offsets[port] = self.getOrDefault(self.offsets, port) + value

    def set_motor_dps(self, port, value):
        self.update()
        self.dps[port] = value

    def update(self):
        dT = self.lastUpdate.update(time())

        for motor in self.dps.keys():
            self.encoders[motor] = self.getOrDefault(self.encoders, motor) + self.dps[motor] * random.uniform(FRICTION, 1) * dT

    def get_sensor(self, port):
        # method is not static because the distance simulation should be edited to be more sophisticated
        return random.randint(MIN_DISTANCE, MAX_DISTANCE)
