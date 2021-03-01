from multiprocessing import Value


class DeltaVal:
    def __init__(self, val=0):
        self.val = Value('d', val)

    def update(self, new):
        old = self.val.value
        self.val.value = new
        return new - old

    def reset(self):
        self.val.value = 0
