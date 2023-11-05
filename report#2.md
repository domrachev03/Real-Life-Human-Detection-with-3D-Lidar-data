# Real Life Human Detection with 3D Lidar data
[The project](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data.git) is aimed to detect a human using data from the 3D lidar. The calculations would be performed in real time on a single board computer.

## Project Participants
1. Domraсhev Ivan, i.domrachev@innopolis.university
2. Eremov Artur, a.eremov@innopolis.university
3. Kiselyov Ivan, i.kiselyov@innopolis.university


# List of completed tasks for last submission
- test Human Detection in Point Cloud Data 
- control software for ackerman model car
- creating dataset with real camera and lidar from car model
- detecting of Humans on camera by YOLO for creating dataset
- detecting of Human coordinates in the video


### test Human Detection in Point Cloud Data

#### Overview

In this project, we aim to detect the presence of humans in point cloud data. Point cloud data is a collection of points in a 3D space, each with its X, Y, and Z coordinates. These data points can be generated using various methods, including LiDAR sensors.

#### Objectives

- Load and preprocess point cloud data.
- Convert point cloud data into a voxelized format suitable for deep learning.
- Design and train a 3D Convolutional Neural Network (CNN) for human detection.
- Evaluate the trained model's performance.

#### Data Preprocessing

The raw point cloud data, obtained from the KITTI dataset, is in `.bin` format. Each file contains a sequence of floating-point numbers, representing the X, Y, Z coordinates and intensity of each point.

##### Voxelization

To make the data suitable for deep learning, we convert the point cloud into a voxel grid format. This process, called voxelization, divides the 3D space into a fixed number of equally-sized cubes (or voxels). If one or more points from the point cloud fall inside a voxel, we mark that voxel as filled.

For this project, we use a voxel grid of size \(32 	imes 32 	imes 32\).

#### Model Design

We employ a simple 3D CNN architecture for this task:

1. A 3D convolutional layer with 32 filters, followed by a ReLU activation.
2. Max pooling with a size of \(2 	imes 2 	imes 2\).
3. Another 3D convolutional layer with 64 filters, followed by a ReLU activation.
4. Max pooling with a size of \(2 	imes 2 	imes 2\).
5. A dense layer with 128 neurons and ReLU activation.
6. A final dense layer with a single neuron and sigmoid activation, which outputs the probability of human presence.

#### Model Training

The model was trained using the binary cross-entropy loss and the Adam optimizer. Training was conducted for 5 epochs, with the model achieving a training accuracy of 100%. This perfect accuracy suggests that the model was able to fit the training data well, given the simplistic labeling heuristic and the small dataset size.

#### Saving and Loading the Model

Once trained, the model's weights were saved to a file named `simple_3dcnn_weights.pth`. This allows for the trained model to be loaded and reused in future without the need for retraining.

#### Results and Evaluation

The trained model was evaluated on the training samples. The predictions were binary, indicating the presence or absence of a human. The model achieved an accuracy of 100% on the training samples.

#### Conclusion

We successfully developed a pipeline for detecting humans in point cloud data using a 3D CNN. The trained model demonstrated high accuracy on the training dataset. However, for real-world applications, a more representative labeling method and a diverse dataset are crucial. Furthermore, the model's performance should be evaluated on a separate test set to assess its generalization capabilities.

#### Future Work

- Use a more representative labeling method.
- Employ regularization techniques and data augmentation to improve generalization.
- Evaluate the model's performance on a larger and more diverse dataset.



### Control software for ackerman model car

#### Overview
The objective of this task was to develop robust control software for an Ackerman model car. The software needed to ensure precise steering and speed control to navigate through predetermined paths.
At start we only had [simulator](https://github.com/PurplePegasuss/agilex_scout_sim) for this robot.

#### Objectives
- Develop a user interface to manually control the Ackerman model car.
- Implement PID control algorithms to manage steering and throttle.
- Integrate sensors' feedback for real-time adjustments.

#### Methodology
We designed a modular control system with interfaces for manual input, sensor data integration, and actuator control. The PID controllers were tuned in a simulated environment before deployment.

#### Results and Evaluation
The control software was tested in a controlled environment with successful navigation through a series of complex paths.
![](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/img/lidar.gif)

#### Conclusion
The control software is capable of handling basic navigation tasks and responds well to manual inputs and sensor feedback.
![](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/img/rosgraph.png)

#### Future Work
- Enhance the control algorithms with machine learning techniques for adaptive control.
- Test the software in more dynamic and unpredictable environments.

### Creating dataset with real camera and lidar from car model

#### Overview
This task involved creating a comprehensive dataset with synchronized data from a camera and Lidar mounted on a car model.

#### Objectives
- Capture and synchronize Lidar and camera data.
- Label the dataset for future use in machine learning tasks.

#### Methodology
A data acquisition system was set up to capture high-fidelity Lidar and visual data. We developed a synchronization mechanism to align the data streams accurately.

#### Results and Evaluation
The dataset is not completely created because we much time spend to create robot control algorithms



### Detecting of Humans on camera by YOLO for creating dataset

#### Overview
We aimed to detect humans in video streams using the YOLO (You Only Look Once) algorithm to create an annotated dataset for training detection models.

#### Objectives
- Implement the YOLO algorithm to detect humans in video data.
- Generate annotated frames for dataset creation.

#### Methodology
The YOLO algorithm was fine-tuned for human detection on our specific video data. An automated annotation pipeline was established to label the detected humans in each frame.

#### Results and Evaluation
The YOLO algorithm achieved a high detection rate with minimal false positives, resulting in a robust dataset for model training.
![](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/img/human_detection_beta.jpg)
![](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/img/human_detection_beta1.jpg)

#### Conclusion
The annotated dataset can significantly reduce the time required for manual labeling and increase the efficiency of training detection models.

#### Future Work
- Improve the precision of detection in varied lighting conditions.
- Expand the dataset to include more diverse human poses and groupings.

### Detecting Human coordinates in the video

#### Overview
The task focused on identifying precise human coordinates in video frames to track movements and interactions over time.

#### Objectives
- Develop an algorithm to pinpoint human coordinates in video frames.
- Track the movement of individuals across frames.

#### Methodology
A combination of detection algorithms and tracking heuristics was employed to locate and follow individuals in a sequence of video frames.

#### Results and Evaluation
The system was capable of tracking individuals with a high degree of accuracy, maintaining identity across multiple frames.

#### Conclusion
The tracking algorithm provides a solid foundation for applications requiring human movement analysis and behavior prediction.

#### Future Work
- Integrate machine learning to improve tracking robustness.
- Test the system in crowded and complex scenes for scalability.



# Future work for last 3 weeks
- Collect our own dataset with varying numbers of people in different conditions
- Test the model's performance, and further train/teach our model for this task
- Deploy everything on the robot, optimize for memory and performance
- Try to give the robot commands to follow a person, and see what comes of it