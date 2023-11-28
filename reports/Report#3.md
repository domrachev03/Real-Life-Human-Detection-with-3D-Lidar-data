# Real Time Human Detection with 3D Lidar data
The project ([github](](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data.git))) is aimed to detect a human using data from the 3D lidar. The calculations would be performed in real time on a single board computer.

> Note. The report contains lots of gif files for better visualization. Therefore, we encorage to go through markdown version of the [report](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/Report%233.md).

**Project Participants**
1. Domraсhev Ivan, i.domrachev@innopolis.university. ROS & Detection developer
2. Eremov Artur, a.eremov@innopolis.university.      Detection & Simulation developer
3. Kiselyov Ivan, i.kiselyov@innopolis.universit.    Scripts & Detection developer.


# Overview of project timeline
- Investigate the existing solutions for real-time human detection based on lidar data.
- Implement the background infrastructure for the robot, and collect the dataset from the lidar.
- Adjust the chosed model parameters to improve the performance.
- Run the model on the collected dataset, and also evaluate its performance on the onboard computer.


## Overview of existing solutions
The detailed report on existing solutions could be found [here](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/reports/Report%231.md).

In summary, we tested three different approaches:
1. **Voxelization-based detection** ([paper](https://www.mdpi.com/1424-8220/23/10/4720), [code](https://github.com/domrachev03/LiDAR_Tracking_3D/tree/c6e36c287c771da163eb9b53d4543a5c9c4a0041)). This approach choose the regions of interest, voxelize them and runs human classification. Despite promising performance metrics, the implementation required structured pointcloud to build normals for voxelization. The current setup could not provide that, so our team decided to omit that approach
2. **Unsupervised segmentation** ([code](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar)). This approach extracted features using K-D tree based pointcloud processing, and then extracted features in unsupervised manner. Unfortunately, this algorithm is designed for tracking of any moving objects, and hence it produced many false positive results. Moverover, the actual tests showed extremely poor result.
<center>
    <figure>
    <img src="img/kf_tracking.gif" width=600 alt="my alt text"/>
    <figcaption>Unsupervised segmentation approach demo. As one could see, the performance in unsatisfactory</figcaption>
    </figure>
</center>

3. **Clustering + Classification** ([Paper](https://www.researchgate.net/publication/331283709_A_portable_three-dimensional_LIDAR-based_system_for_long-term_and_wide-area_people_behavior_measurement), [code](https://github.com/koide3/hdl_people_tracking/tree/2d125167ca2b0935b8075032b943edf11d996788)). This algorithm performs [Haselich's clustering technique](https://userpages.uni-koblenz.de/~agas/Documents/Haeselich2014CBP.pdf) to detect human candidate clusters, and then applies [Kidono's person classifier](https://www.aisl.cs.tut.ac.jp/~jun/pdffiles/kidono-iv2011.pdf) to eliminate false detections. 
   The results on our dataset were significanly better, than for the previous model. On top of that, the code is very flexible and has many parameters to configure. Therefore, we decided to proceed with this algorithm. 

## Robot infrastructure
The detailed report on existing solutions could be found [here](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/reports/Report%232.md).

The initial robot was designed to perform autonomous motion. To solve this task, it needs to firstly create a map (it's called mapping) and also localize himself inside it (it's called localization). As a result, the odometry of the robot is obtained.
<center>
    <figure>
    <img src="img/system_pipeline.png" width=1000 alt="my alt text"/>
    <figcaption>Approximate system pipeline. Orange rectangle depicts mapping, green -- localization, and blue -- human tracking</figcaption>
    </figure>
</center>

Obviously, it's important to have an access to the odometry, since we need to know the shift of the robot w.r.t. the tracked person. Hence, our team accessed the implemented Simultaneous Localization and Mapping algorithm [gmapping](https://openslam-org.github.io/gmapping.html). 

<center>
    <figure>
    <img src="img/lidar.gif" width=600 alt="my alt text"/>
    <figcaption>Data from the SLAM algorithm. It creates a 2D map of the area and finds itself within it.</figcaption>
    </figure>
</center>

## Model optimization
The performance optimization turned out to be quite simple, and consisted of two major parts:
1. **Floor removal algorithm.** The main problem of working with lidar data is very big amount of points, most of which are redundant. To cope with this issue, most lidar data pipelines contain *floor removal* algorithm, and our system would utilize it as well. We stopped on the algorithm [patchwork-plusplus](https://github.com/url-kaist/patchwork-plusplus), since it is SOTA in floor removal at the moment.

<center>
    <figure>
    <img src="img/floor-removal.gif" width=600 alt="my alt text"/>
    <figcaption>Result of floor removal algorithm. Red points are detected floor points, and blue one are the others.</figcaption>
    </figure>
</center>

2. **Parameter tuning.** The human detection algorithm has many different parameters. Meanwhile most of them were great out-of-box, we decided to change the parameters responsible for downsampling and resolution so that they produce less load on the system.
   
## Evaluation on the dataset
### The datasest
To evaluate the performance of the model, the test data was collected from the robot sensors and lidar data. The Real Sense camera was attached to record the data in one of the directions. To "label" the data, we planned to launch the YOLO detection model, then triangulate the predictions from both of the cameras and those obtain the coordinate of the person. 

<center>
    <figure>
    <img src="img/ShowDataset.gif" width=600 alt="my alt text"/>
    <figcaption>The dataset (the update rate is slowed by a factor of 40) with two people in the frame.</figcaption>
    </figure>
</center>

However, there are two issues with such approach:
1. The camera sees the people only in a very small range and in one direction
2. The YOLO model does not track the people, but only detects them.

On top of that, we didn't have enough time with robot to collect enough data. Summing this up, we decided to collect only two data entries: one with one person, and another with two people.

All the data we managed to collect could be found [here](https://drive.google.com/drive/folders/1oSeZWjjiLW-ZYMtTdAdhg11_YxoUQRsg?usp=sharing).

### Results
We made two runs of the algorithm: locally on our machine, just to evaluate accuracy, and then on the robot itself.

To make long things short, both runs were successful, and the robot managed to detect and track all the humans presented on the scene all the time. You can see the sample gifs below (or use the links: [one person](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/reports/img/OwnSimulator.gif), [two people](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/reports/img/FinalDetection.gif)) 

<center>
    <figure>
    <img src="img/OwnSimulator.gif" width=600 alt="my alt text"/>
    <figcaption>Human detection for one human scene. The robot clearly see the human clearly.</figcaption>
    </figure>
</center>

<center>
    <figure>
    <img src="img/OwnSimulator.gif" width=600 alt="my alt text"/>
    <figcaption>Human detection for several people. The network is tracking the furthest person. The data was collected from the robot and launched in the simulation later.</figcaption>
    </figure>
</center>

> Note: to visualize data from the robot, we first recorded it and then launched on the modified version of [lidar data simulation](https://github.com/ipipos56/Lidar-Simulator). Unfortunately, due to agreement with robot owner, this data would not be shared publicly.

As for the performance, the [Jetson Nano 8Gb](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) was able to run the algorithm at rate 5 Hz with SLAM and other algorithms running on the background and the average CPU load around 80-90%. This rate is considered sufficient, however it might be dropped lower by manipulating the parameters of the human detection algorithm. 

> Note: despite using Jetson Nano 8 Gb, the algorithm still utilized only CPU power because of the implementation of the algorithm. Due to the nature of the solution, it does not run deep neural networks, so it would not benefit from transporting onto the GPU. However, this is still an open question

# Summary
During implementation of this project, our team:
1. Investigated existing implementations algorithms for human detection based on lidar data. It turns out there are not so many, especially one integrated in the Robotics Operating System (ROS) middleware.
2. Adapted existing software from the robot to be able to obtain data about its location, the map and lidar points. All the software was implemented using ROS.
3. Made an attempt to collect the dataset from the robot. Attached the camera, made the script to collect the data, and recorded two sample scenes with one and two people.
4. Adjusted the algorithm to make the algorithm faster. Considering the fact that the algorithm was already designed for online usage, this was not very challenging task.
5. Run the algorithm on the collected data both in "simulation" (locally) and on the robot (but still using prerecorded datasets). In both cases the model succeeded in detecting all the people on the scene.

The future improvements one might consider are:
1. Run proper evaluation of the algorithm on the datasets like [KITTI](https://www.cvlibs.net/datasets/kitti) and/or others.
2. Add the tracking task for the robot based on the tracking of chosen person (plan path to the chosen point, namely nearby the person).
3. Try to optimize the model even further without drastic accuracy loss. Our team failed to find a way to speed up the computation further without loss of the accuracy.
