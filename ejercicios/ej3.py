import matplotlib.pyplot as plt
import numpy as np

from functions.dibrobot import dibrobot
from functions.functions import norm_pi, hom, loc, plotLocVector
from functions.simubot import simubot

from classes.Map import Map

##########################################################################
# CONSTANTS
##########################################################################
# Control constants
K_RO    = 1
K_BETA  = 1.8
K_ALPHA = 1.8
assert K_RO > 0 and K_BETA > 0 and K_ALPHA - K_RO > 0

# Speed constants
V_MAX = 17
W_MAX = np.pi/4

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
    if v > V_MAX: v = V_MAX
    if w > W_MAX: w = W_MAX
    return v, w

def areEqual(u, v, allowedError = 1):
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
WxR = hom(np.array([0,0,0]))
WxM = hom(np.array([30,30,np.pi/2]))

circularMotion = [15, -0.15] # v/w = R, R = 100 for left or -100 for right
assert circularMotion[0]/circularMotion[1] == -100

picture = Map()
picture.update(loc(WxR))
##############################
showStatus(loc(WxR),loc(WxM))
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
    picture.update(loc(WxR))
    ##############################
    print("----------------------------------------------")
    showStatus(loc(WxR),loc(WxM))
    ##############################

picture.block()
