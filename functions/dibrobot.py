import matplotlib.pyplot as plt
import numpy as np


def dibrobot(loc_eje, c, scale=1):
    """
    Plot robot in loc_eje with colour c and scale size
    """
    largo = 200 / 2 * scale
    corto = 121 / 2 * scale
    descentre = -largo

    trasera_dcha = np.array([-largo, -corto, 1])
    trasera_izda = np.array([-largo, corto, 1])
    delantera_dcha = np.array([largo, -corto, 1])
    delantera_izda = np.array([largo, corto, 1])
    frontal_robot = np.array([largo, 0, 1])
    tita = loc_eje[2]
    Hwe = np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
                    [np.sin(tita), np.cos(tita), loc_eje[1]],
                    [0, 0, 1]])
    Hec = np.array([[1, 0, descentre],
                    [0, 1, 0],
                    [0, 0, 1]])
    extremos = np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
    robot = np.dot(Hwe, np.dot(Hec, np.transpose(extremos)))
    plt.plot(robot[0, :], robot[1, :], c)
