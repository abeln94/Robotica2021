import time

START = None


def perf_counter_exact():
    """ Return the time elapsed since the script which includes this module is thrown and this function is invoked """
    global START
    if START is None:
        START = time.perf_counter()
        return 0
    return time.perf_counter() - START
