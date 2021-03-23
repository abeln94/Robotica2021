import numpy as np


def sigmoid(x, alpha):
    """
    Sigmoid function implemented as hyperbolic tangent of x*alpha, where x is the independent variable and alpha the curve's slope measure
    :param x: independent variable
    :param alpha: measure of the curve's slope, where a high value means a high slope, and a low one a low slope
    :return: a value between -1 and 1, the hyperbolic tangent of x*alpha
    """
    return np.tanh(x * alpha)


def logistic(x, alpha, beta):
    """
    Sigmoid function implemented as logistic function of -alpha * (x - beta), where x is the independent variable, alpha the curve's slope measure and beta an offset over the axis value
    :param x: independent variable
    :param alpha: measure of the curve's slope, where a high value means a high slope, and a low one a low slope
    :param beta: offset over the axis value
    :return: a value between 0 and 1, the logistic function of -alpha * (x - beta)
    """
    return 1 / (1 + pow(np.e, -alpha * (x - beta)))
