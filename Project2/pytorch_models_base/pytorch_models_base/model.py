import torch
import torch.nn as nn

class CNNModel(nn.Module):
    def __init__(self, channel, classes, drop):
        super(CNNModel, self).__init__()
        self.classes = classes
        self.cnn_layers = nn.Sequential(
            # Block 1
            # Mini 1
            nn.Conv2d(classes, 32, 3),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            # Mini 2
            nn.Conv2d(64, 64, 3),
            nn.BatchNorm2d(64),
            nn.MaxPool2d(2),
            nn.ReLU(),
        )
        self.fc = nn.Sequential(
            nn.Linear(12544, 4600),
            nn.ReLU(),
            nn.Dropout(drop),
            nn.Linear(6400, 3200),
            nn.ReLU(),
            nn.Dropout(drop),
            nn.Linear(3200, 1500),
            nn.ReLU(),
            nn.Dropout(drop),
            nn.Linear(1500, channel),
        )

        self.softmax = nn.Softmax(dim=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Perform the forward pass with the net

        Args:
        -   x: the input image [Dim: (N,C,H,W)]
        Returns:
        -   y: the output (raw scores) of the net [Dim: (N,15)]
        """
        model_output = None
        ############################################################################
        # Student code begin
        ############################################################################
        # normalize input
        model_output = self.cnn_layers(x)
        # model_output = torch.flatten(model_output, start_dim=1)
        model_output = self.fc(model_output)
        if self.classes > 1:
            model_output = self.softmax(model_output)
        ############################################################################
        # Student code end
        ############################################################################

        return model_output
