import numpy as np


def sigmoid(x, alpha=3, beta=0, asPositive=False):
    """
    Sigmoid function implemented as hyperbolic tangent of x*alpha, where x is the independent variable and alpha the curve's slope measure
    :param x: independent variable
    :param alpha: measure of the curve's slope, where a high value means a high slope, and a low one a low slope
    :return: a value between 0 and 1, the hyperbolic tangent of x*alpha
    """
    val = np.tanh(x * alpha + beta)
    return val if not asPositive else (val + 1) / 2
