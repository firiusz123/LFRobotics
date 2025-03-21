import torch
import torch.nn as nn

class SimpleNet(nn.Module):
    def __init__(self):
        super(SimpleNet, self).__init__()
        self.n = 6
        # Convolutional layers
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, padding=1,stride=1)  # Input: 3 channels (RGB)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1,stride=1)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, padding=1,stride=1)
        self.conv4 = nn.Conv2d(128,256,kernel_size=3,padding=1,stride=1)
        self.conv5 = nn.Conv2d(256,512,kernel_size=3,padding=1,stride=1)

        # Max Pooling
        self.pool = nn.MaxPool2d(2, 2)

        # Fully connected layers
        self.fc1 = nn.Linear(512 * (100//2**self.n) * (440//2**self.n), 1)  # Adjust for size after pooling
        self.fc2 = nn.Linear(512, 1)  # Output single error value
        self.lerelu = nn.ReLU()
        self.dropout = nn.Dropout(p=0.1)  # 50% dropout
    def forward(self, x):
        x = self.pool(x)
        #print(x.shape)
        x = self.pool(self.lerelu(self.conv1(x)))  # First conv + ReLU + Pool
        x = self.dropout(x)
        #print(x.shape)
        x = self.pool(self.lerelu(self.conv2(x)))  # Second conv + ReLU + Pool
        x = self.dropout(x)
        #print(x.shape)
        x = self.pool(self.lerelu(self.conv3(x)))  # Third conv + ReLU + Pool
        x = self.dropout(x)
        #print(x.shape)
        x = self.pool(self.lerelu(self.conv4(x)))
        x = self.dropout(x)
        #print(x.shape)
        x = self.pool(self.lerelu(self.conv5(x)))
        #print(x.shape)
        
        x = x.view(-1, 512 * (100//2**self.n) * (440//2**self.n))  # Flatten for fully connected layers
        x = self.fc1(x)
    
        return x