import matplotlib.pyplot as plt
import numpy as np

from functions.dibrobot import dibrobot
from functions.functions import norm_pi, hom, loc
from functions.simubot import simubot

# path initialization
points = [(2, 3, np.deg2rad(90)),
          (6, 4, np.deg2rad(45)),
          (10, 5, np.deg2rad(-45)),
          (7, -3, np.deg2rad(180)),
          (2, 3, np.deg2rad(90))]
robot = points[0]
current = 1

for a, b in zip(points[:-1], points[1:]):
    print(np.linalg.norm([a[0] - b[0], a[1] - b[1]]), norm_pi(a[2] - b[2]), norm_pi(np.arctan2(a[1] - b[1], a[0] - b[0]) + np.pi))

# graph initialization
plt.ion()
f, (ax1, ax2) = plt.subplots(2, 1, num='ej2 velocities')
plt.figure('ej2 map')
plt.plot([p[0] for p in points], [p[1] for p in points], '*', color='red')
plt.gca().set_aspect(True)
plt.show(block=False)

vs = []
ws = []

while True:
    # get relative position
    WxG = hom(np.array(points[current]))
    WxR = hom(np.array(robot))
    GxR = np.linalg.inv(WxG) @ WxR
    dx, dy, th = loc(GxR)

    # get polar coordinates
    p = np.linalg.norm([dx, dy])
    b = norm_pi(np.arctan2(dy, dx) + np.pi)
    a = b - th

    # calculate velocities
    v = 0.34 * p
    w = 1.28 * a + 1.56 * b
    # w = 1.28 * a + 1.28 / 2 * b

    # plot velocities
    for data, next, label, ax in [(vs, v, 'v', ax1), (ws, w, 'w', ax2)]:
        data.append(next)
        ax.clear()
        ax.set_ylabel(label)
        ax.plot(range(len(data)), data)

    # update robot
    dibrobot(robot, 'green', 0.01)
    robot = simubot([v, w], np.array(robot), 0.5)
    dibrobot(robot, 'blue', 0.01)

    # refresh map
    plt.gcf().canvas.draw()
    plt.gcf().canvas.flush_events()

    # check goal
    if p < 0.1 and b < 0.1 and a < 0.1:
        current += 1
        if current == len(points):
            break

plt.show(block=True)
