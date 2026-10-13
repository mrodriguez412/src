import numpy as np
import cv2
import os

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import albumentations as A


class PNGReader(Dataset):
    def __init__(self, label_file, image_root_directory, is_ce=True):
        self.label_path = label_file
        self.image_root_directory = image_root_directory
        self.is_ce = is_ce
        self.transform = None
        self.load_data()
        self.compute_mean()

    def add_transform(self,transform):
        self.transform = transform

    def load_data(self):
        # LOAD TRAIN
        x_path, self.y = self.read_file(self.label_path)
        self.x = self.load_x(x_path, self.image_root_directory)
        
    def read_file(self, filepath):
        x_path = []
        y = []
        with open(filepath) as f:
            line = f.readline()
            while line:
                data = line.strip().split(' ')
                x_path.append(data[0])
                if self.is_ce:
                    y.append(int(data[1]))
                else:
                    y.append(float(data[2]))
                line = f.readline()
        if self.is_ce:
            y = np.array(y)
        else:
            y = np.array(y, dtype=np.float32)
        return x_path, y

    def load_x(self, path_list, root):
        x = []
        for path in path_list:
            path = os.path.join(root, path)
            raw = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            image = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
            # x.append(image.astype(np.float32)/255)
            sx, sy = raw.shape[0:2]
            x.append(cv2.resize(raw,(0,0),fx=32.0/sx,fy=32.0/sy, interpolation = cv2.INTER_AREA).astype(np.float32)/255.)
        return x

    def compute_mean(self):
        self.mean = np.mean(np.mean(np.mean(np.array(self.x), axis = 0), axis = 0),axis = 0)
        self.std = np.std(np.std(np.std(np.array(self.x), axis = 0), axis = 0), axis = 0)


    def __len__(self):
        return self.y.shape[0]

    def __getitem__(self, idx):
        # Load image
        image = self.x[idx].copy()

        # Apply transforms
        if self.transform:
            augmented = self.transform(image=image)
            image = augmented['image']

        return image, self.y[idx].copy()

