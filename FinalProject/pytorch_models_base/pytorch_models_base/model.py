import torch
import torch.nn as nn


def conv_block(c_in, c_out):
    # Two 3x3 convs (padding keeps the size), then halve the resolution.
    return nn.Sequential(
        nn.Conv2d(c_in, c_out, 3, padding=1, bias=False),
        nn.BatchNorm2d(c_out),
        nn.ReLU(inplace=True),
        nn.Conv2d(c_out, c_out, 3, padding=1, bias=False),
        nn.BatchNorm2d(c_out),
        nn.ReLU(inplace=True),
        nn.MaxPool2d(2),
    )


class CNNModel(nn.Module):
    # Small VGG-style network for 32x32 images (~0.3M parameters).
    # The old version flattened 64x14x14 features into 6400/3200/1500-unit FC layers (~100M parameters)
    # and overfitted badly: train acc 87% vs val acc ~60%, val loss rising after epoch ~14.
    # Here features are extracted by conv layers and averaged per channel (global average pooling),
    # so the classifier head is tiny and the network works with any input size.
    def __init__(self, channel, classes, drop):
        super().__init__()
        self.classes = classes
        self.cnn_layers = nn.Sequential(
            conv_block(channel, 32),     # 32x32 -> 16x16
            conv_block(32, 64),          # 16x16 -> 8x8
            conv_block(64, 128),         # 8x8   -> 4x4
            nn.AdaptiveAvgPool2d(1),     # (N, 128, 4, 4) -> (N, 128, 1, 1)
        )
        self.fc = nn.Sequential(
            nn.Flatten(),                # (N, 128)
            nn.Dropout(drop),
            nn.Linear(128, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(drop),
            nn.Linear(64, classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.cnn_layers(x)
        return self.fc(x)               # raw scores (logits)
