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

_FRICTION = 1  # 0.975

_encoder = "_encoder"
_dps = "_dps"
_offset = "_offset"
_value = "_value"


class BrickPi3:
    PORT_1 = "PORT_1"
    PORT_2 = "PORT_2"
    PORT_3 = "PORT_3"
    PORT_A = "PORT_A"
    PORT_B = "PORT_B"
    PORT_C = "PORT_C"

    class SENSOR_TYPE:
        TOUCH = "Touch"
        NXT_ULTRASONIC = "Ultrasonic"

        _MOTOR = "Motor"

    def __init__(self):
        m = Manager()
        self.ports = m.dict()
        self.data = m.dict()
        self.lastUpdate = SyncDeltaVal()

        self.finished = Value('b', False)
        self.p = Process(target=self._ui)
        self.p.start()

    ########## general ##########

    def set_sensor_type(self, port, type):
        self.ports[port] = type
        self.data[port + _value] = 255

    def update(self):
        dT = self.lastUpdate.update(time())

        for port, type in self.ports.items():
            if type == self.SENSOR_TYPE._MOTOR:
                self.data[port + _encoder] += self.data[port + _dps] * random.uniform(_FRICTION, 1) * dT

    def reset_all(self):
        self.finished.value = True
        self.lastUpdate.reset()
        for key in self.data.keys():
            self.data[key] = 0

    def get_sensor(self, port):
        return self.data[port + _value]

    ########## Motor ##########

    def _assertMotor(self, port):
        if port in self.ports:
            assert self.ports[port] == self.SENSOR_TYPE._MOTOR
        else:
            self.ports[port] = self.SENSOR_TYPE._MOTOR
            self.data[port + _encoder] = 0.0
            self.data[port + _dps] = 0.0
            self.data[port + _offset] = 0.0

    def get_motor_encoder(self, port):
        self.update()
        self._assertMotor(port)
        return int(self.data[port + _encoder] - self.data[port + _offset])

    def offset_motor_encoder(self, port, value):
        self.update()
        self._assertMotor(port)
        self.data[port + _offset] += value

    def set_motor_dps(self, port, value):
        self.update()
        self._assertMotor(port)
        self.data[port + _dps] = value

    ########## UI ##########

    def _ui(self):

        createdPorts = []

        # Create the Window
        window = sg.Window('Robot simulator controller', [[]], size=(512, 512))

        while not self.finished.value:
            event, values = window.read(timeout=0)
            if event == sg.WIN_CLOSED:  # if user closes window or clicks cancel
                break

            for port, type in self.ports.items():

                if port not in createdPorts:
                    createdPorts.append(port)
                    if type == self.SENSOR_TYPE._MOTOR:
                        window.extend_layout(window, [[
                            sg.Text(port + ": Motor:"),
                            sg.RealtimeButton("<", key=port + "<"),
                            sg.Text(key=port + "º", auto_size_text=False, size=(3, 1)),
                            sg.RealtimeButton(">", key=port + ">"),
                            sg.VerticalSeparator(),
                            sg.RealtimeButton("-", key=port + "-"),
                            sg.Text(key=port + "dps", auto_size_text=False, size=(5, 1)),
                            sg.RealtimeButton("+", key=port + "+"),
                        ]])
                    if type == self.SENSOR_TYPE.NXT_ULTRASONIC:
                        window.extend_layout(window, [[
                            sg.Text(port + ": Ultrasonic:"),
                            sg.Slider(range=(0, 255), default_value=self.data[port + _value], orientation='horizontal', key=port),
                        ]])

                if type == self.SENSOR_TYPE._MOTOR:
                    window[port + "º"].update(str(self.get_motor_encoder(port) % 360) + "º")
                    window[port + "dps"].update(str(self.data[port + _dps]) + " dps")
                    if event == port + "<":
                        self.data[port + _encoder] += 1
                    if event == port + ">":
                        self.data[port + _encoder] -= 1
                    if event == port + "-":
                        self.data[port + _dps] += 1
                    if event == port + "+":
                        self.data[port + _dps] -= 1

                if type == self.SENSOR_TYPE.NXT_ULTRASONIC:
                    if port in values:
                        self.data[port + _value] = values[port]
