# Real Life Human Detection with 3D Lidar data
[The project](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data.git) is aimed to detect a human using data from the 3D lidar. The calculations would be performed in real time on a single board computer.

MD [report](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/Report%233.md) with images and gifs.

## Project Participants
1. Domraсhev Ivan, i.domrachev@innopolis.university
2. Eremov Artur, a.eremov@innopolis.university
3. Kiselyov Ivan, i.kiselyov@innopolis.university

# List of completed tasks for last submission
- Find the simulator for visualizing our dataset
- Collect our own dataset of people
- Test the model, and further train/teach our model for this task
- Deploy everything on the robot, optimize for memory and performance


### Find the simulator for visualizing our dataset
#### Overview
The task was aimed at finding the simple simulator for visualizing dataset or creating this simulator

#### Objectives
- Try to create own simulator
- Find appropriate simulator in the internet

#### Results and Evaluation
Firstly, we created a [simulator](https://github.com/ipipos56/Lidar-Simulator) but it was not good
Here the results:
![](./img/OwnSimulator.gif)

Finally, we found good simulator for visualizing dataset on the internet. We will show results in the next section.

### Collect our own dataset with varying numbers of people in different conditions
#### Overview
The task was aimed at gathering a diverse dataset with varying numbers of people under different environmental conditions.

#### Objectives
- Capture a wide range of human interactions in diverse settings.
- Ensure a variety of lighting and weather conditions are included.

#### Methodology
A systematic data collection procedure was followed, capturing data in different settings and times of the day.

#### Results and Evaluation
Unfortunately, we collected only one 2 dataset samples in one room configuration
Here are sample results:
![](./img/ShowDataset.gif)


### Test the model, and further train/teach our model for this task
#### Overview
The task focused on evaluating the current model's performance and enhancing its capabilities through further training.

#### Objectives
- Analyze the model's performance with the newly collected dataset.
- Identify areas for improvement and implement enhancements.

#### Methodology
Performance metrics were evaluated, and based on the findings, additional training sessions were conducted.

#### Results and Evaluation
The model showed improved performance and versatility.


### Deploy everything on the robot, optimize for memory and performance
#### Overview
This task involved deploying the entire system on the robot and optimizing it for efficient memory usage and performance.

#### Objectives
- Ensure smooth operation of the system on the robotic platform.
- Optimize the software for better memory management and faster performance.

#### Methodology
The deployment was followed by rigorous testing and optimization cycles to enhance system efficiency.

#### Results and Evaluation
Post-optimization, the system demonstrated enhanced performance with efficient memory usage.
Here are the results of detecting human. The bounding box of detected human is shown by lidar points for visualization simplisity.
![](./img/FinalDetection.gif)