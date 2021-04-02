import time

from classes.DeltaVal import DeltaVal


class Periodic:
    """
    For loops that needs to be run periodically.

    Use as
    periodic = Periodic(delay)  # delay can be omitted
    while periodic(condition): # condition can be omitted, defaults to True
        pass # this line will run exactly each delay seconds

    """

    def __init__(self, delay=0.1):
        """
        :param delay: delay between loop executions, in seconds
        """
        self.delay = delay
        self.delta = None

    def __call__(self, condition=True):
        """
        Note 1: On first call no delay is performed (initialization)
        Note 2: If the condition is True, no delay is performed (so the loop exits immediately)
        :param condition: condition for the loop
        :return: condition
        """
        if condition:
            # wait only when the condition is true

            if self.delta is None:
                # first call, init
                self.delta = DeltaVal(time.clock())
            else:
                # next call, wait
                delay = self.delay - self.delta.update(time.clock())
                if delay > 0: time.sleep(delay)

        else:
            # reset itself on exit
            self.delta = None

        return condition
