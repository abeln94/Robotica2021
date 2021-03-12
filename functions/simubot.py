import numpy as np

from functions.functions import loc, hom, norm_pi


def simubot(vc, xWR, T):
    """
    Simulate robot's movement with vc=[v,w] linear/angular velocity (rad/s) during T second starting from location xWR
    """
    if vc[1] == 0:  # w=0
        xRk = np.array([vc[0] * T, 0, 0])
    else:
        R = vc[0] / vc[1]
        dtitak = vc[1] * T
        titak = norm_pi(dtitak)
        xRk = np.array([R * np.sin(titak), R * (1 - np.cos(titak)), titak])

    xWRp = loc(np.dot(hom(xWR), hom(xRk)))  # nueva localización xWR
    return xWRp
