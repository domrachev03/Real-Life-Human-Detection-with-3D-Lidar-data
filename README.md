# Real Life Human Detection with 3D Lidar data
[The project](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data.git) is aimed to detect a human using data from the 3D lidar. The calculations would be performed in real time on a single board computer.

## Project Participants
1. Domraсhev Ivan, i.domrachev@innopolis.university
2. Eremov Artur, a.eremov@innopolis.university
3. Kiselyov Ivan, i.kiselyov@innopolis.university

## Motivation
The project is industry motivated, since this is very close to a real-life goal of one of the start-ups in the Innopolis University. The data from the model would be postprocessed and published to a navigation algorithm, which would allow to autonomous vehicle follow the person. The lidar is used for other means, and it would be fortunate if it would allow not to install the camera specifically for this task

## Challenges
1. *Lack of resources.* Not only all computations would be performed on the single board Jetson Xavier in real time, but also it would share computational resources with other algorithms: SLAM, path planning etc. Therefore, the clever resource management is a cornerstone of the project.
2. *Different environment.* The robot is supposed to perform well under both indoor and outdoor environment, in crowded places (more on that later) etc. The goal of the resulting work is to create a robust solution, which would be able to handle all of them with minimal finetuning
3. *Tracking certain person.* From the motivation it becomes clear, that the task is not only to detect all the humans on the image, but also to track certain one (so that the vehicle could follow him). This would probably be beyond the scope of Machine Learning methods, but still an interesting challange to solve

## Setup
The current hardware setup is following:
1. Mobile Platform [AgileX Scout Mini](https://global.agilex.ai/products/scout-mini)
2. 3D lidar [Velodyne Puck](https://velodynelidar.com/products/puck/)
3. Single Board computer [NVIDIA Jetson AGX Xavier series](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-xavier-series/)

Also, the Robotics Operating System (ROS) enviroment with other algorithms for autonomous navigation are already implements

## Roadmap
- [X] Designing a pipeline
- [X] Researching for existing solutions
- [X] Integrating a solution with ROS environment
- [ ] Collecting the dataset
- [ ] Implementing human detection pipeline
- [ ] Implementing human following
- [ ] Optimizing the performance

