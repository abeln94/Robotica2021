import time

START = None


def perf_counter_exact():
    global START
    if START is None:
        START = time.perf_counter()
        return START
    return time.perf_counter() - START
