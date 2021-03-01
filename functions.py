import numpy as np


def hom(x: np.ndarray) -> np.ndarray:
    """
    transform from localization coordinates to the homography matrix
    :param x: localization coordinates
    :return: homography matrix
    """
    assert x.shape == (3,)
    xp, yp, t = x

    return np.array([
        [np.cos(t), -np.sin(t), xp],
        [np.sin(t), np.cos(t), yp],
        [0, 0, 1]
    ])


def loc(T: np.ndarray) -> np.ndarray:
    """
    transform from the homography matrix to localization coordinates
    :param T: homography matrix
    :return: localization coordinates
    """
    assert T.shape == (3, 3)
    assert T[0, 0] - T[1, 1] < 1e-8
    assert T[0, 1] - -T[1, 0] < 1e-8
    assert T[2, 0] == T[2, 1] == 0
    assert T[2, 2] == 1

    return np.array([T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])])


def norm_pi(angle: float) -> float:
    """
    Normalize the angle to be in range [-pi, pi]
    :param angle: input angle
    :return: same angle but between [-pi, pi]
    """
    while angle > np.pi: angle -= 2 * np.pi
    while angle < -np.pi: angle += 2 * np.pi
    return angle
