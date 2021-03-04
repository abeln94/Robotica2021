class DeltaVal:
    """
    For delta updates of a value (can be shared between processes)
    """

    def __init__(self, val=0):
        self.val = val

    def update(self, new):
        old = self.val
        self.val = new
        return new - old

    def reset(self):
        self.val = 0
