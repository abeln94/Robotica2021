import matplotlib.pyplot as plt
import numpy as np

from functions.dibrobot import dibrobot
from functions.simubot import simubot


def sensor(robot):
    # Estimated sonar restrictions
    d_max = 30
    d_min = 5
    angle_max = np.deg2rad(30)

    # If the th has too deviation, we use the max angle
    angle = min(angle_max, robot[2]) if robot[2] > 0 else max(-angle_max, robot[2])

    d = robot[1] / np.cos(angle)

    return min(d_max,(max(d_min, d)))


# Parameters
k1 = 0.015
k2 = -0.20

v = 1
w_max = np.deg2rad(10)
dc_lat = 10  # Objetive parameter
dc_front = 5  # To stop
x_map_size = 50
y_map_size = 20

# robot initialization
robot = [0, 13, np.deg2rad(30)]

# graph initialization
plt.ion()
f, (ax1, ax2) = plt.subplots(2, 1, num='ej4 velocities')
plt.figure('ej4 map')
plt.plot([0, x_map_size, x_map_size], [0, 0, y_map_size])
plt.gca().set_aspect(True)
plt.show(block=False)

vs = []
ws = []

d = robot[1] / np.cos(robot[2])  # Distance
while True:


    d_old = d
    d = sensor(robot) # Medida sensor
    #d = robot[1] / np.cos(robot[2])
    d_var = d - d_old

    # calculate angular velocity
    w = k1 * (dc_lat - d) + k2 * d_var

    # limitate velocity
    w = min(w_max, w) if w > 0 else max(-w_max, w)

    print(robot[1])
    print((dc_lat - d))
    print(d_var)
    print(w)
    print("")

    # plot velocities
    for data, next, label, ax in [(vs, v, 'v', ax1), (ws, w, 'w', ax2)]:
        data.append(next)
        ax.clear()
        ax.set_ylabel(label)
        ax.plot(range(len(data)), data)

    # update robot
    dibrobot(robot, 'green', 0.01)
    robot = simubot([v, w], np.array(robot), 0.75)
    dibrobot(robot, 'blue', 0.01)

    # refresh map
    plt.gcf().canvas.draw()
    plt.gcf().canvas.flush_events()

    # check goal
    if robot[0] >= x_map_size - dc_front:
        break

plt.show(block=True)
