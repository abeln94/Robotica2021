import matplotlib.pyplot as plt
import numpy as np

from functions.dibrobot import dibrobot
from functions.functions import norm_pi, hom, loc, plotLocVector
from functions.simubot import simubot

##########################################################################
# CONSTANTS
##########################################################################
# Control constants
K_RO    = 0.3
K_BETA  = 0.5
K_ALPHA = 0.4
assert K_RO > 0 and K_BETA > 0 and K_ALPHA - K_RO > 0

# Speed constants (length in milimeters)
V_MAX = 3000 # m/s
W_MAX = 3    # rad/s

##########################################################################
# AUXILIAR FUNCTIONS
##########################################################################

def cartesian2polar(cartesianLocation):
    x, y, th = cartesianLocation
    ro = np.sqrt(x**2 + y**2)
    beta = norm_pi(np.arctan2(y,x) + np.pi)
    alpha = beta - th
    return np.array([ro, beta, alpha])

def getSpeed(ro, alpha, beta):
    v = K_RO * ro
    w = K_ALPHA * alpha + K_BETA * beta
    # Speed saturation
    v = min(v, V_MAX) if v > 0 else max(v, -V_MAX)
    w = min(w, W_MAX) if w > 0 else max(w, -W_MAX)
    return v, w

def areEqual(u, v, allowedError = 200):
    return all([abs(dif) <= allowedError for dif in u - v])

####################################
# DEBUG
####################################
def showStatus(locWxR, locWxM):
    plotLocVector(locWxR,"ROBOT")
    plotLocVector(locWxM,"MOVIL")
    print(areEqual(locWxR, locWxM))

##########################################################################
# MAIN
##########################################################################

# G goal, R robot => Coordenadas del robot con respecto al goal
WxR = hom(np.array([0, 0, 0]))
WxM = hom(np.array([500, 500, np.pi / 2]))

circularMotion = [100, -0.01]  # v/w = R, R = 10000 for left or -10000 for right
assert circularMotion[0] / circularMotion[1] == -10000

# graph initialization
plt.ion()
f, (ax1, ax2) = plt.subplots(2, 1, num='ej3 velocities')
plt.figure('ej3 map')
plt.gca().set_aspect(True)
plt.show(block=False)

vs = []
ws = []

##############################
dibrobot(loc(WxR), 'blue', 1)
dibrobot(loc(WxM), 'red', 1)
showStatus(loc(WxR), loc(WxM))
##############################
while not areEqual(loc(WxR), loc(WxM)):
    # Simulate the movil's motion
    WxM = hom(simubot(circularMotion, loc(WxM), 1))
    # Get the proper movement of the robot to reach the movil
    MxR = np.linalg.inv(WxM) @ WxR
    MpR = cartesian2polar(loc(MxR))
    v, w = getSpeed(*MpR)
    # Simulate the robot's motion
    WxR = hom(simubot([v,w], loc(WxR), 1))
    ##############################
    # plot velocities
    for data, next, label, ax in [(vs, v, 'v', ax1), (ws, w, 'w', ax2)]:
        data.append(next)
        ax.clear()
        ax.set_ylabel(label)
        ax.plot(range(len(data)), data)

    dibrobot(loc(WxR), 'blue', 1)
    dibrobot(loc(WxM), 'red', 1)

    # refresh map
    plt.gcf().canvas.draw()
    plt.gcf().canvas.flush_events()

    print("----------------------------------------------")
    showStatus(loc(WxR),loc(WxM))
    ##############################

plt.show(block=True)
