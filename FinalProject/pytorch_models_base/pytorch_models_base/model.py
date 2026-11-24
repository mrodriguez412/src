import torch
import torch.nn as nn

class CNNModel(nn.Module):
    def __init__(self, channel, classes, drop):
        super().__init__()
        self.classes = classes
        self.cnn_layers = nn.Sequential(
            # Block 1
            nn.Conv2d(channel, 32, 3),   # 3 -> 32,  32x32 -> 30x30
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.Conv2d(32, 64, 3),        # 32 -> 64, 30x30 -> 28x28
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(2),             # 28x28 -> 14x14
        )
        self.fc = nn.Sequential(
            nn.Flatten(),                # (N, 64, 14, 14) -> (N, 12544)
            nn.Linear(64 * 14 * 14, 6400),
            nn.ReLU(),
            nn.Dropout(drop),
            nn.Linear(6400, 3200),
            nn.ReLU(),
            nn.Dropout(drop),
            nn.Linear(3200, 1500),
            nn.ReLU(),
            nn.Dropout(drop),
            nn.Linear(1500, classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.cnn_layers(x)
        return self.fc(x)               # raw scores (logits)