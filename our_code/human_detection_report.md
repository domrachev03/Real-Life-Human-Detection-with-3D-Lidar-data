

# Human Detection in Point Cloud Data

## Overview

In this project, we aim to detect the presence of humans in point cloud data. Point cloud data is a collection of points in a 3D space, each with its X, Y, and Z coordinates. These data points can be generated using various methods, including LiDAR sensors.

## Objectives

- Load and preprocess point cloud data.
- Convert point cloud data into a voxelized format suitable for deep learning.
- Design and train a 3D Convolutional Neural Network (CNN) for human detection.
- Evaluate the trained model's performance.

## Data Preprocessing

The raw point cloud data, obtained from the KITTI dataset, is in `.bin` format. Each file contains a sequence of floating-point numbers, representing the X, Y, Z coordinates and intensity of each point.

### Voxelization

To make the data suitable for deep learning, we convert the point cloud into a voxel grid format. This process, called voxelization, divides the 3D space into a fixed number of equally-sized cubes (or voxels). If one or more points from the point cloud fall inside a voxel, we mark that voxel as filled.

For this project, we use a voxel grid of size \(32 	imes 32 	imes 32\).

## Model Design

We employ a simple 3D CNN architecture for this task:

1. A 3D convolutional layer with 32 filters, followed by a ReLU activation.
2. Max pooling with a size of \(2 	imes 2 	imes 2\).
3. Another 3D convolutional layer with 64 filters, followed by a ReLU activation.
4. Max pooling with a size of \(2 	imes 2 	imes 2\).
5. A dense layer with 128 neurons and ReLU activation.
6. A final dense layer with a single neuron and sigmoid activation, which outputs the probability of human presence.

## Model Training

The model was trained using the binary cross-entropy loss and the Adam optimizer. Training was conducted for 5 epochs, with the model achieving a training accuracy of 100%. This perfect accuracy suggests that the model was able to fit the training data well, given the simplistic labeling heuristic and the small dataset size.

## Saving and Loading the Model

Once trained, the model's weights were saved to a file named `simple_3dcnn_weights.pth`. This allows for the trained model to be loaded and reused in future without the need for retraining.

## Results and Evaluation

The trained model was evaluated on the training samples. The predictions were binary, indicating the presence or absence of a human. The model achieved an accuracy of 100% on the training samples.

## Conclusion

We successfully developed a pipeline for detecting humans in point cloud data using a 3D CNN. The trained model demonstrated high accuracy on the training dataset. However, for real-world applications, a more representative labeling method and a diverse dataset are crucial. Furthermore, the model's performance should be evaluated on a separate test set to assess its generalization capabilities.

## Future Work

- Use a more representative labeling method.
- Employ regularization techniques and data augmentation to improve generalization.
- Evaluate the model's performance on a larger and more diverse dataset.

