import torch
import torch.nn as nn

class SimpleNet(nn.Module):
    def __init__(self):
        super(SimpleNet, self).__init__()
        self.n = 4
        # Convolutional layers
        self.conv1 = nn.Conv2d(3, 64, kernel_size=3, padding=1,stride=1)  # Input: 3 channels (RGB)
        self.conv2 = nn.Conv2d(64, 128, kernel_size=3, padding=1,stride=1)
        self.conv3 = nn.Conv2d(128, 256, kernel_size=3, padding=1,stride=1)
        self.conv4 = nn.Conv2d(256,512,kernel_size=3,padding=1,stride=1)

        # Max Pooling
        self.pool = nn.MaxPool2d(2, 2)

        # Fully connected layers
        self.fc1 = nn.Linear(512 * (480//2**self.n) * (640//2**self.n), 1)  # Adjust for size after pooling
        self.fc2 = nn.Linear(512, 1)  # Output single error value

    def forward(self, x):
        x = self.pool(torch.relu(self.conv1(x)))  # First conv + ReLU + Pool
        x = self.pool(torch.relu(self.conv2(x)))  # Second conv + ReLU + Pool
        x = self.pool(torch.relu(self.conv3(x)))  # Third conv + ReLU + Pool
        x = self.pool(torch.relu(self.conv4(x)))
        
        x = x.view(-1, 512 * (480//2**self.n) * (640//2**self.n))  # Flatten for fully connected layers
        x = self.fc1(x)
    
        return x