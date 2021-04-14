import matplotlib.pyplot as plt
import numpy as np

from functions.dibrobot import dibrobot
from functions.functions import norm_pi, hom, loc
from functions.simubot import simubot




# Parameters
"""
k1 = 0.03
k2 = -0.4
w_max = np.deg2rad(30)
dc_lat = 5
dc_front = 10 
"""

k1 = 0.015
k2 = -0.15
v = 1

w_max = np.deg2rad(15)
dc_lat = 10
dc_front = 5
x_map_size = 60

# robot initialization
robot = [0, 12, np.deg2rad(30)]


# graph initialization
plt.ion()
f, (ax1, ax2, ax3) = plt.subplots(3, 1, num='ej4')
plt.plot([0, x_map_size, x_map_size],[0, 0, x_map_size / 3])
plt.gca().set_aspect(True)
plt.show(block=False)


vs = []
ws = []

d = robot[1] / np.cos(robot[2]) # Distancia
while True:
    # get relative position

    # TODO: funcion sensor. Limitar sensor??
    d_old = d
    d = robot[1] / np.cos(robot[2]) # Medida sensor
    d_var = d - d_old

    # calculate angular velocity
    w = k1 * (dc_lat - d) + k2 * d_var

    # limitate velocity
    w = min(w_max, w) if w > 0 else max(-w_max, w)

    # plot velocities
    for data, next, label, ax in [(vs, v, 'v', ax1), (ws, w, 'w', ax2)]:
        data.append(next)
        ax.clear()
        ax.set_ylabel(label)
        ax.plot(range(len(data)), data)

    # update robot
    dibrobot(robot, 'green', 0.01)
    robot = simubot([v, w], np.array(robot), 1)
    dibrobot(robot, 'blue', 0.01)

    # refresh map
    plt.gcf().canvas.draw()
    plt.gcf().canvas.flush_events()

    print(robot[1])

    # check goal
    if robot[0] > x_map_size - dc_front:
        break

plt.show(block=True)
