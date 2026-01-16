import torch
import torchvision
import torchvision.transforms as transforms
# We must import the Net class structure to load the model pickle, 
# or ensure it is available in the namespace.
from train_model import Net 

def test():
    device = torch.device("cpu")

    # Transforms (must match training)
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5,), (0.5,))
    ])

    # Load Test Data
    testset = torchvision.datasets.MNIST(root='./data', train=False,
                                         download=True, transform=transform)
    testloader = torch.utils.data.DataLoader(testset, batch_size=64, shuffle=False)

    # Load Model
    try:
        # weights_only=False permite cargar el objeto completo (Net) que guardaste
        net = torch.jit.load('my_network.pt')
        net.to(device)
        net.eval()
    except FileNotFoundError:
        print("Error: my_network.pt not found. Run train_model.py first.")
        return

    correct = 0
    total = 0
    
    # No gradient needed for testing
    with torch.no_grad():
        for data in testloader:
            images, labels = data
            images, labels = images.to(device), labels.to(device)
            
            outputs = net(images)
            _, predicted = torch.max(outputs.data, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()

    accuracy = 100 * correct / total
    print(f'Accuracy of the network on the 10000 test images: {accuracy:.2f}%')

    if accuracy > 95:
        print("SUCCESS: Model accuracy is > 95%.")
    else:
        print("WARNING: Model accuracy is below 95%. Consider training for more epochs.")

if __name__ == "__main__":
    test()