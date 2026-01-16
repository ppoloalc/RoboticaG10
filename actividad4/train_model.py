import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms
import torch.nn.functional as F

# --- Network Architecture ---
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        # 1st Conv Layer: 1 input channel (grayscale), 32 output channels, 3x3 kernel
        self.conv1 = nn.Conv2d(1, 32, kernel_size=3, padding=1)
        # 2nd Conv Layer: 32 input channels, 64 output channels, 3x3 kernel
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
        
        # Pooling layer
        self.pool = nn.MaxPool2d(2, 2)
        
        # Fully Connected Layers
        # Image reduces to 7x7 after two poolings (28->14->7)
        self.fc1 = nn.Linear(64 * 7 * 7, 128)
        self.fc2 = nn.Linear(128, 10) # Output layer (10 digits)

    def forward(self, x):
        # Conv1 -> Relu -> MaxPool
        x = self.pool(F.relu(self.conv1(x)))
        # Conv2 -> Relu -> MaxPool
        x = self.pool(F.relu(self.conv2(x)))
        
        # Flatten
        x = x.view(-1, 64 * 7 * 7)
        
        # FC Layers
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

def train():
    # Setup device
    device = torch.device("cpu")
    print(f"Training on: {device}")

    # Transforms
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5,), (0.5,)) # Normalize to [-1, 1]
    ])

    # Load Data
    trainset = torchvision.datasets.MNIST(root='./data', train=True,
                                          download=True, transform=transform)
    trainloader = torch.utils.data.DataLoader(trainset, batch_size=100, shuffle=True)

    # Initialize Network
    net = Net().to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(net.parameters(), lr=0.001)

    # Training Loop (5 Epochs is usually enough for >98% on MNIST)
    epochs = 7
    for epoch in range(epochs):
        running_loss = 0.0
        for i, data in enumerate(trainloader, 0):
            inputs, labels = data
            inputs, labels = inputs.to(device), labels.to(device)

            optimizer.zero_grad()
            outputs = net(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()

            running_loss += loss.item()
            if i % 200 == 199:
                print(f'[Epoch {epoch + 1}, Batch {i + 1}] loss: {running_loss / 200:.3f}')
                running_loss = 0.0

    print('Finished Training')

    # Save the model
    example_imput = torch.rand(1, 1, 28, 28)
    traced_model = torch.jit.trace(net, example_imput)
    traced_model.save('my_network.pt')
    
if __name__ == "__main__":
    train()