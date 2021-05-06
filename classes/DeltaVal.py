from multiprocessing import Value


class DeltaVal:
    """
    For delta updates of a value (can not be shared between processes)
    """

    def __init__(self, val=0):
        self.val = val

    def update(self, new):
        """ Returns the difference between this 'new' parameter and the previous one """
        old = self.val
        self.val = new
        return new - old


class SyncDeltaVal:
    """
    For delta updates of a value (can be shared between processes)
    Synced version
    """

    def __init__(self, val=0):
        self.val = Value('d', val)

    def update(self, new):
        """ Returns the difference between this 'new' parameter and the previous one """
        old = self.val.value
        self.val.value = new
        return new - old
