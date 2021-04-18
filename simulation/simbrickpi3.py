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
import numpy as np

from classes.DeltaVal import SyncDeltaVal

sg.theme('DarkAmber')  # Add a touch of color

_ICON = 20  # icons size in pixels


class _Motor:
    _encoder = "_encoder"
    _dps = "_dps"
    _offset = "_offset"

    _FRICTION = 1  # 0.975

    @staticmethod
    def init(port, data):
        data[port + _Motor._encoder] = 0.0
        data[port + _Motor._dps] = 0.0
        data[port + _Motor._offset] = 0.0

    @staticmethod
    def read(port, data):
        return int(data[port + _Motor._encoder] - data[port + _Motor._offset])

    @staticmethod
    def offset_encoder(port, data, value):
        data[port + _Motor._offset] += value

    @staticmethod
    def set_dps(port, data, value):
        data[port + _Motor._dps] = value

    @staticmethod
    def update(port, data, dT):
        data[port + _Motor._encoder] += data[port + _Motor._dps] * random.uniform(_Motor._FRICTION, 1) * dT

    @staticmethod
    def initUI(port, data):
        return [[
            sg.Text(port + ": Motor:"),
            sg.RealtimeButton("<", key=port + "<"),
            sg.Graph(canvas_size=(_ICON, _ICON), graph_bottom_left=(0, 0), graph_top_right=(_ICON, _ICON), key=port + "º"),
            sg.RealtimeButton(">", key=port + ">"),
            sg.VerticalSeparator(),
            sg.RealtimeButton("-", key=port + "-"),
            sg.Text(key=port + "dps", auto_size_text=False, size=(8, 1)),
            sg.RealtimeButton("+", key=port + "+"),
        ]]

    @staticmethod
    def updateUI(event, values, window, port, data):

        graph = window[port + 'º']
        graph.DrawCircle((_ICON / 2, _ICON / 2), _ICON / 2, fill_color='white')
        angle = np.deg2rad(int(data[port + _Motor._offset]))
        graph.DrawLine((_ICON / 2, _ICON / 2), (_ICON / 2 + _ICON / 2 * np.cos(angle), _ICON / 2 + _ICON / 2 * np.sin(angle)), color='blue')
        angle = np.deg2rad(_Motor.read(port, data))
        graph.DrawLine((_ICON / 2, _ICON / 2), (_ICON / 2 + _ICON / 2 * np.cos(angle), _ICON / 2 + _ICON / 2 * np.sin(angle)), color='red')

        window[port + "dps"].update("{:.2f} dps".format(data[port + _Motor._dps]))
        if event == port + "<":
            data[port + _Motor._encoder] += 5
        if event == port + ">":
            data[port + _Motor._encoder] -= 5
        if event == port + "-":
            data[port + _Motor._dps] += 5
        if event == port + "+":
            data[port + _Motor._dps] -= 5


class _Touch:
    @staticmethod
    def init(port, data):
        data[port] = 0

    @staticmethod
    def update(port, data, dT):
        pass

    @staticmethod
    def read(port, data):
        return data[port]

    @staticmethod
    def initUI(port, data):
        return [[
            sg.Text(port + ": Touch: "),
            sg.RealtimeButton("(+)", key=port),
        ]]

    @staticmethod
    def updateUI(event, values, window, port, data):
        data[port] = event == port


class _Ultrasonic:
    @staticmethod
    def init(port, data):
        data[port] = 255

    @staticmethod
    def update(port, data, dT):
        pass

    @staticmethod
    def read(port, data):
        return data[port]

    @staticmethod
    def initUI(port, data):
        return [[
            sg.Text(port + ": Ultrasonic:"),
            sg.Slider(range=(0, 255), default_value=data[port], orientation='horizontal', key=port),
        ]]

    @staticmethod
    def updateUI(event, values, window, port, data):
        if port in values:
            data[port] = values[port]


class _Light:
    @staticmethod
    def init(port, data):
        data[port] = 1750

    @staticmethod
    def update(port, data, dT):
        pass

    @staticmethod
    def read(port, data):
        return data[port]

    @staticmethod
    def initUI(port, data):
        return [[
            sg.Text(port + ": Light:"),
            sg.Graph(canvas_size=(_ICON, _ICON), graph_bottom_left=(0, 0), graph_top_right=(_ICON, _ICON), key=port + "#"),
            sg.Slider(range=(0, 3500), default_value=data[port], orientation='horizontal', key=port),
        ]]

    @staticmethod
    def updateUI(event, values, window, port, data):
        graph = window[port + '#']
        graph.DrawRectangle((0, 0), (_ICON, _ICON), fill_color='#{0:02x}{0:02x}{0:02x}'.format(255 - int(_Light.read(port, data) / 3500 * 255)))

        if port in values:
            data[port] = values[port]


class BrickPi3:
    PORT_1 = "PORT_1"
    PORT_2 = "PORT_2"
    PORT_3 = "PORT_3"
    PORT_A = "PORT_A"
    PORT_B = "PORT_B"
    PORT_C = "PORT_C"

    class SENSOR_TYPE:
        TOUCH = _Touch
        NXT_ULTRASONIC = _Ultrasonic
        NXT_LIGHT_ON = _Light
        NXT_LIGHT_OFF = _Light

    def __init__(self):
        m = Manager()
        self.ports = m.dict()  # used ports
        self.data = m.dict()  # all the data (in a synced dict)
        self.lastUpdate = SyncDeltaVal()

        self.finished = Value('b', False)
        self.p = Process(target=self._ui)
        self.p.start()

    ########## general ##########

    def set_sensor_type(self, port, type):
        self.ports[port] = type
        type.init(port, self.data)

    def get_sensor(self, port):
        return self.ports[port].read(port, self.data)

    def reset_all(self):
        # this is more of a 'close_all' than a 'reset_all'
        self.finished.value = True
        self.p.join()

    def _update(self):
        dT = self.lastUpdate.update(time())

        for port, type in self.ports.items():
            type.update(port, self.data, dT)

    ########## Motor ##########

    def _assertMotor(self, port):
        if port not in self.ports:
            # not yet, init
            self.set_sensor_type(port, _Motor)
        assert self.ports[port] == _Motor

    def get_motor_encoder(self, port):
        self._update()
        self._assertMotor(port)
        return _Motor.read(port, self.data)

    def offset_motor_encoder(self, port, value):
        self._update()
        self._assertMotor(port)
        _Motor.offset_encoder(port, self.data, value)

    def set_motor_dps(self, port, value):
        self._update()
        self._assertMotor(port)
        _Motor.set_dps(port, self.data, value)

    ########## UI ##########

    def _ui(self):

        createdPorts = []  # already initialized ports

        # Create the Window
        window = sg.Window('Robot simulator controller', [[]], size=(512, 200))

        while not self.finished.value:
            event, values = window.read(timeout=0)  # ui magic

            if event == sg.WIN_CLOSED:
                # if user closes window or clicks cancel, exit
                break

            # check each port
            for port, type in sorted(self.ports.items()):

                if port not in createdPorts:
                    # new one, create first
                    createdPorts.append(port)
                    window.extend_layout(window, type.initUI(port, self.data))

                # update
                type.updateUI(event, values, window, port, self.data)
