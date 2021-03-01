import random
from enum import Enum, auto
from multiprocessing import Manager
from time import time

from DeltaVal import DeltaVal

FRICTION = 1  # 0.975


class BrickPi3:
    class SENSOR_TYPE(Enum):
        TOUCH = auto()

    PORT_1 = "PORT_1"
    PORT_A = "PORT_A"
    PORT_B = "PORT_B"
    PORT_C = "PORT_C"

    def __init__(self):
        m = Manager()
        self.encoders = m.dict()
        self.offsets = m.dict()
        self.dps = m.dict()
        self.lastUpdate = DeltaVal()

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
