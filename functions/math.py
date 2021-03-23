import numpy as np


def sigmoid(x, alpha):
    """
    Sigmoid function implemented as hyperbolic tangent of x*alpha, where x is the independent variable and alpha the curve's slope measure
    :param x: independent variable
    :param alpha: measure of the curve's slope, where a high value means a high slope, and a low one a low slope
    :return: a value between -1 and 1 if asPositive is false, else 0,1, the hyperbolic tangent of x*alpha
    """
    return np.tanh(x * alpha)


def logistic(x, alpha, beta):
    return 1 / (1 + pow(np.e, -alpha * (x - beta)))
