
import venv_hack

from parser import ParseArgs
from reader import PNGReader
from model import CNNModel
from sampler import UniformSampler

import numpy as np
import os
import torch
from torch.optim import Adam
import torch.nn as nn
from torchinfo import summary
import matplotlib.pyplot as plt
from torch.utils.tensorboard import SummaryWriter
from torch.utils.data import Dataset, DataLoader
import math

import albumentations as A


class Train:
    def __init__(self):
        self.settings = ParseArgs()
        # Set device to use CUDA
        if not self.settings.nocuda and torch.cuda.is_available():
            self.device = torch.device('cuda')
            print("Using CUDA for Torch")
        else:
            self.device = torch.device('cpu')
            print("Using CPU for Torch")
        # Part 1 uses CE Loss
        if self.settings.loss_func == "ce":
            print("Using CrossEntropy loss")
            self.train_loss = nn.CrossEntropyLoss()
            self.val_loss = nn.CrossEntropyLoss()
            self.is_cross_entropy = True
        # Part 2 uses HuberLoss for a regression problem
        else:
            print("Using HuberLoss(mse) loss")
            self.train_loss = nn.HuberLoss(delta=0.05)
            self.val_loss = nn.HuberLoss(delta=0.05)
            self.is_cross_entropy = False
        self.train_dataset = PNGReader(self.settings.train_file, self.settings.train_root, self.is_cross_entropy)
        print(f"Train dataset: {len(self.train_dataset)} images")
        self.val_dataset = PNGReader(self.settings.val_file, self.settings.val_root, self.is_cross_entropy)
        print(f"Val dataset: {len(self.val_dataset)} images")

        # Training transforms - comprehensive augmentation
        train_transforms = A.Compose([
            A.ColorJitter(brightness_range=(0.8, 1.2), contrast_range=(0.8, 1.2), saturation_range=(0.8, 1.2), hue_range=(-0.1, 0.1), p=0.8),
            # A.Normalize(mean=self.train_dataset.mean, std=self.train_dataset.std),
            A.ToTensorV2(),
        ])
        self.train_dataset.add_transform(train_transforms)

        # Validation transforms - deterministic
        val_transforms = A.Compose([
            # A.Normalize(mean=self.train_dataset.mean, std=self.train_dataset.std),
            A.ToTensorV2(),
        ])
        self.val_dataset.add_transform(val_transforms)

        self.train_loader = DataLoader(self.train_dataset, batch_size=self.settings.bs, shuffle=True)
        self.val_loader = DataLoader(self.val_dataset, batch_size=self.settings.bs, shuffle=False)

        self.model = CNNModel(3, self.settings.num_class, self.settings.dropout).to(self.device)
        # Need to los mean and std for the model during inference
        print(self.train_dataset.mean, self.train_dataset.std)
        self.optimizer = Adam(self.model.parameters(), lr=self.settings.lr, weight_decay=self.settings.weight_decay)

        # Can still use tensorboard
        self.writer = SummaryWriter()
        # Pytorch model architecture logging
        model_summary = summary(self.model, (self.settings.bs, 3, 32, 32), device=self.device, verbose=0)
        model_summary = str(model_summary)
        self.writer.add_text("model", model_summary)
    

    def calc_accuracy(self, pred, target):
        total = len(target)
        correct = sum(pred.argmax(1) == target)
        return correct / total

    def calc_accuracy_mse(self, pred, target):
        total = len(target)
        correct = sum(abs(pred - target) < 1e-1)
        return correct / total

    def train(self):
        train_losses = []
        val_losses = []
        train_accuracies = []
        val_accuracies = []
        best_acc = 0
        max_epoch = self.settings.max_iter
        x_train_batches_per_epoch = math.ceil(len(self.train_dataset) / self.settings.bs)
        x_val_batches_per_epoch = math.ceil(len(self.val_dataset) / self.settings.bs)
        # Iterate over the number of epochs
        for epoch in range(max_epoch):
            # Set model to eval mode so weights and dropout arn't calculated for train
            self.model.eval()
            batch_loss_val = 0.0
            batch_acc_val = 0.0
            # for all batches in val
            with torch.no_grad():
                first = True
                for x, y in self.val_loader:
                    x = x.to(self.device)
                    y = y.to(self.device)
                    val_pred = self.model(x)
                    if not self.is_cross_entropy:
                        val_pred = val_pred.squeeze(1)
                    if self.is_cross_entropy:
                        val_acc = self.calc_accuracy(val_pred, y)
                    else:
                        val_acc = self.calc_accuracy_mse(val_pred, y)
                    val_loss = self.val_loss(val_pred, y)
                    batch_loss_val += val_loss.item()
                    batch_acc_val += val_acc.item()
                    first = False
            batch_loss_val /= x_val_batches_per_epoch
            batch_acc_val /= x_val_batches_per_epoch
            val_losses.append(batch_loss_val)
            val_accuracies.append(batch_acc_val)

            # allow model weights to update
            self.model.train()
            batch_loss_train = 0.0
            batch_acc_train = 0.0
            first = True
            for x, y in self.train_loader:
                x = x.to(self.device)
                y = y.to(self.device)
                self.optimizer.zero_grad()
                pred = self.model(x)
                if not self.is_cross_entropy:
                    pred = pred.squeeze(1)
                if self.is_cross_entropy:
                    train_acc = self.calc_accuracy(pred, y)
                else:
                    train_acc = self.calc_accuracy_mse(pred, y)
                loss = self.train_loss(pred, y)
                loss.backward()
                self.optimizer.step()
                batch_loss_train += loss.item()
                batch_acc_train += train_acc.item()
                first = False
            batch_loss_train /= x_train_batches_per_epoch
            batch_acc_train /= x_train_batches_per_epoch
            train_losses.append(batch_loss_train)
            train_accuracies.append(batch_acc_train)

            # Log to tensorboard
            self.writer.add_scalar("Loss/train", batch_loss_train, epoch)
            self.writer.add_scalar("Loss/val",   batch_loss_val, epoch)
            self.writer.add_scalar("Acc/train",  batch_acc_train, epoch)
            self.writer.add_scalar("Acc/val",    batch_acc_val, epoch)
            print(f"Epoch: {epoch+1}|{max_epoch} Train Loss: {batch_loss_train} Val Loss {batch_loss_val} Train Acc {batch_acc_train} Val Acc {batch_acc_val}")
            if batch_acc_val >= best_acc:
                best_acc = batch_acc_val
                type_ =  "ce" if self.is_cross_entropy else "mse"
                torch.save(self.model.state_dict(), f"{self.settings.model_dir}/model_{type_}_{round(val_acc.item()*100)}.pt")
        self.writer.add_hparams(
            {
                "lr": self.settings.lr,
                "weight_decay": self.settings.weight_decay,
                "batch_size": self.settings.bs,
                "dropout": self.settings.dropout,
                "iter": self.settings.max_iter,
            },
            {
                "train_acc": train_accuracies[-1],
                "test_acc": val_accuracies[-1],
            },
        )
        # show learning curves and save to disk
        plt.figure()
        plt.title(f"Training and Validation Loss")
        plt.plot(train_losses, label="Training Loss")
        plt.plot(val_losses, label="Validation Loss")
        plt.xlabel("Iter")
        plt.ylabel("Loss")
        plt.legend()
        plt.savefig('loss.png')
        plt.show()
        plt.figure()
        plt.title(f"Training and Validation Accuracy")
        plt.plot(train_accuracies, label="Training Accuracy")
        plt.plot(val_accuracies, label="Validation Accuracy")
        plt.xlabel("Iter")
        plt.ylabel("Accuracy")
        plt.legend()
        plt.savefig('acc.png')
        plt.show()
        self.writer.flush()
        self.writer.close()
        

def main(args=None):
    T = Train()
    T.train()

if __name__ == '__main__':
    main()
