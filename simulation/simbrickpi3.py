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
from multiprocessing import Manager, Value
from multiprocessing.context import Process
from time import time

import PySimpleGUI as sg

from classes.DeltaVal import SyncDeltaVal

sg.theme('DarkAmber')  # Add a touch of color

FRICTION = 1  # 0.975
MIN_DISTANCE = 200  # 20 cm
MAX_DISTANCE = 2000  # 2 m


class BrickPi3:
    class SENSOR_TYPE:
        TOUCH = "Button"
        NXT_ULTRASONIC = "Proximity sensor"

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

        self.finished = Value('b', False, lock=self.lock_odometry)
        self.p = Process(target=self.ui)
        self.p.start()

    def ui(self):
        layout = [
            [sg.Text('Some text on Row 1', key='text')],
            [sg.Text('Motor'), sg.Slider(range=(0, 500), default_value=222, size=(20, 15), orientation='horizontal', font=('Helvetica', 12), key='slider')]
        ]

        # Create the Window
        window = sg.Window('Robot simulator', layout)

        while not self.finished.value:
            event, values = window.read(timeout=200)
            if event == sg.WIN_CLOSED:  # if user closes window or clicks cancel
                break
            self.encoders[self.PORT_C] = values['slider']
            window['text'].update(self.getOrDefault(self.encoders, self.PORT_B))
            window['slider'].update(self.getOrDefault(self.encoders, self.PORT_C))

    def reset_all(self):
        self.finished.value = True
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
