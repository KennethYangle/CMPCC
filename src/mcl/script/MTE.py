import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
from scipy.optimize import linear_sum_assignment
import numpy as np

# Mock dataset
class MultiTargetDataset(Dataset):
    def __init__(self, num_samples, num_targets, time_steps):
        super().__init__()
        self.num_samples = num_samples
        self.num_targets = num_targets
        self.time_steps = time_steps

    def __len__(self):
        return self.num_samples

    def __getitem__(self, idx):
        # Mock data for demonstration purposes
        poses = torch.randn(self.time_steps, 6)  # (time_steps, pose_dim)
        num_detected = np.random.randint(1, self.num_targets + 1)
        image_features = torch.randn(num_detected, 2)  # (num_detected, image_feat_dim)
        positions = torch.randn(self.num_targets, 3)  # Ground truth 3D positions
        return poses, image_features, positions

# Define the model
class MultiTargetEstimatorWithPhysics(nn.Module):
    def __init__(self, num_targets, pose_dim=6, image_feat_dim=2, hidden_dim=128):
        super(MultiTargetEstimatorWithPhysics, self).__init__()
        self.num_targets = num_targets

        # Pose encoder for time-series UAV pose
        self.pose_encoder = nn.LSTM(input_size=pose_dim, hidden_size=hidden_dim, num_layers=2, batch_first=True)

        # Image feature encoder
        self.image_encoder = nn.Sequential(
            nn.Linear(image_feat_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim)
        )

        # Physics-based module for motion constraints
        self.physics_predictor = nn.Sequential(
            nn.Linear(hidden_dim, 6),  # Outputs initial position (x, y, z) and velocity (vx, vy, vz)
        )

        # Final prediction head for positions
        self.position_decoder = nn.Sequential(
            nn.Linear(hidden_dim + 6, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 3)  # Outputs (x, y, z)
        )

    def forward(self, poses, image_features):
        batch_size = poses.size(0)

        # Pose encoding
        pose_encoded, _ = self.pose_encoder(poses)  # (batch_size, time_steps, hidden_dim)
        pose_context = pose_encoded[:, -1, :]  # (batch_size, hidden_dim)

        # Image feature encoding
        image_encoded = self.image_encoder(image_features)  # (batch_size, num_detected, hidden_dim)
        pooled_features = image_encoded.mean(dim=1)  # Symmetric operation (order invariant)

        # Combine pose and image features
        combined_features = torch.cat([pose_context, pooled_features], dim=-1)  # (batch_size, hidden_dim * 2)

        # Predict initial state (position and velocity)
        physics_params = self.physics_predictor(combined_features)  # (batch_size, 6)

        # Decode positions
        positions = self.position_decoder(torch.cat([combined_features, physics_params], dim=-1))  # (batch_size, 3)

        # Replicate for num_targets
        positions = positions.unsqueeze(1).expand(-1, self.num_targets, -1)

        return positions

# Loss function with set matching
def set_loss(predicted, ground_truth):
    batch_size, num_targets, _ = predicted.size()
    loss = 0
    for i in range(batch_size):
        cost_matrix = torch.cdist(predicted[i], ground_truth[i])  # Pairwise distances
        row_ind, col_ind = linear_sum_assignment(cost_matrix.cpu().detach().numpy())
        loss += cost_matrix[row_ind, col_ind].sum()
    return loss / batch_size

# Training loop
def train_model(model, dataloader, optimizer, epochs):
    model.train()
    for epoch in range(epochs):
        epoch_loss = 0
        for poses, image_features, positions in dataloader:
            poses = poses.float()
            image_features = image_features.float()
            positions = positions.float()

            optimizer.zero_grad()
            predictions = model(poses, image_features)
            loss = set_loss(predictions, positions)
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()
        
        print(f"Epoch {epoch + 1}/{epochs}, Loss: {epoch_loss / len(dataloader)}")

# Inference (usage) loop
def evaluate_model(model, dataloader):
    model.eval()
    with torch.no_grad():
        for poses, image_features, _ in dataloader:
            poses = poses.float()
            image_features = image_features.float()

            predictions = model(poses, image_features)
            print(f"Predicted positions: {predictions}")

# Main script
if __name__ == "__main__":
    num_samples = 100
    num_targets = 5
    time_steps = 10
    batch_size = 8
    epochs = 5

    dataset = MultiTargetDataset(num_samples, num_targets, time_steps)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    model = MultiTargetEstimatorWithPhysics(num_targets)
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    print("Starting training...")
    train_model(model, dataloader, optimizer, epochs)

    print("\nEvaluating model...")
    evaluate_model(model, dataloader)
