import numpy as np

class UniformSampler:
    def __init__(self, Dataset):
        self.DS = Dataset
        self.TB = None
        self.TEB = None
        self.VB = None

    def sample(self, x, y, bs):
        # Shuffle before each epoch
        s = list(range(x.shape[0]))
        np.random.shuffle(s)
        x = x.copy()[s]
        y = y.copy()[s]
        
        # Split into batches
        bx = []
        by = []
        for i in range(int(x.shape[0]/bs)):
            bx.append(x[i*bs:(i+1)*bs,:,:,:])
            by.append(y[i*bs:(i+1)*bs])
        bx = np.array(bx)
        by = np.array(by)
        
        # Run
        for i in zip(bx,by):
            yield i[0].astype(np.float32), i[1]

    def sample_train_batch(self, bs):
        try:
            return next(self.TB)
        except:
            self.TB = self.sample(self.DS.x_train, self.DS.y_train, bs)
            return next(self.TB)

    def sample_val_batch(self, bs):
        try:
            return next(self.VB)
        except:
            self.VB = self.sample(self.DS.x_val, self.DS.y_val, bs)
            return next(self.VB)

