# Report 1
# Human Detection in Point Cloud Data 

## Link to [github](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data.git)


## Progress

- Explored and loaded different datasets
- Recorded chosen datasets 
- Created a ROS environment for checking algorithm work with all visualizations
- Deployed floor removal algorithm
- Tested several existing algorithms
- Made an attempt to write our own Neural Network 

## Datasets

Now we do not have our own dataset because at this moment we do not work with real robot. However, there are many datasets publicly available. We've considered two of them:
1. [NuScenes](https://www.nuscenes.org/)
2. [KITTY](https://www.cvlibs.net/datasets/kitti/) 
We decided to stop on KITTY, since it provides raw syncronised lidar data. The datasets are stored [there](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/tree/master/existing_models/datasets). 

### Rosbag
[Here](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/tree/master/existing_models/datasets/KITTI_dataset) we took code for automatically load several KITTY scenes and recording lidar points to rosbag. It will be used for comfortable integration with ROS environment.
### Dependencies
   * ROS 1 (was tested on ROS Noetic, ubuntu 20.04)
   * `unzip` (`sudo apt install unzip`)
   * `kitti2bag` (installed in the notebook)
### Usage
Open the notebook, write required dataset names (could be explored [there](https://www.cvlibs.net/datasets/kitti/raw_data.php)) and path to python version with ROS installed

## ROS environment
Based on [this](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar/tree/27db548ea51d8faa498c9a2492b172219d6a56fb) project we made a ROS environment for us for visualizing all our data and for checking algorithms. It automatically launches the rviz for visualization and one could specify the name of the loaded rosbag file and the NN to launch.
### Dependencies
   * ROS 1 (full desktop)
### Usage
``` bash
cd ~/.../LidarTracking3D
catkin make # Requires installation of all dependencies from packages, see below
source devel/setup.{bash/zsh/...}
roslaunch lidar_tracker_validation validate_tracker.launch model_name:={"kf_tracker/Track3D/kf_tracker"} rosbag_filename={rosbag_file.bag}
```

## Floor removal
The lidar data often requires a preprocessing. The most common one -- floor removal. The great algorithm was found [there](https://github.com/url-kaist/patchwork-plusplus). You can find the prerequisites, building and other information in the original reposityry.

![GIF](img/floor-removal.gif)

## Algorithms
We managed to test three different algorithms for the people tracking:
**1. Efficient Detection and Tracking of Human using 3D LiDAR Sensors**
   * [Paper](https://www.mdpi.com/1424-8220/23/10/4720)
   * [Code](https://github.com/domrachev03/LiDAR_Tracking_3D/tree/c6e36c287c771da163eb9b53d4543a5c9c4a0041)

This approach selects regions of interest, and then utilizes them to voxelize the image and then segment and classify it via voting among several other classifiers. Combined, the model is very lightweight and runs real time even on weak hardware, like single board computers.

The main disadvantage of this approach is requirement of [structured pointcloud](https://kaolin.readthedocs.io/en/latest/notes/spc_summary.html) to execute. The KITTI dataset provides only unstructured laser scan and performing Lidar pointcloud structurization is a non-trivial task. Hence, our team decided to omit it till structured pointcloud would be obtained.

**2. Multiple objects detection, tracking and classification from LIDAR scans/point-clouds.**
   * [Code](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar)

![GIF](http://wiki.ros.org/multi_object_tracking_lidar?action=AttachFile&do=get&target=multi-object-tracking-lidar-demo.gif)
This approach utilises K-D tree based pointcloud processing for object feature detection, then passes it to unsuprevised euclidean cluster extraction for segmentation and finally implements stable tracking with ensemble of Kalman filters.

Despite great performance on the presented references, this model performed poorly on chosen KITTI samples. Also, the implementation does not provide clear way for configuring parameters.

![GIF](img/kf_tracking.gif)

**3. A portable three-dimensional LIDAR-based system for long-term and wide-area people behavior measurement**
   * [Paper](https://www.researchgate.net/publication/331283709_A_portable_three-dimensional_LIDAR-based_system_for_long-term_and_wide-area_people_behavior_measurement)
   * [Code](https://github.com/koide3/hdl_people_tracking/tree/2d125167ca2b0935b8075032b943edf11d996788)
> Note: code requires lots of dependencies, most of them are mentioned as submodules in the folder `hdl_people_tracking_dependencies`.

This algorithm performs Haselich's clustering technique to detect human candidate clusters, and then applies Kidono's person classifier to eliminate false detections.

The results on our dataset were significanly better, than for the previous model. On top of that, the code is very flexible and has many parameters to configure. 

![GIF](img/hdl_tracking.gif)

## Conclusion
We successfully developed a pipeline for loading the data and testing it on different datasets. There are not many models available publicly, we chose the best ones and examined them on the datasets. 


## Appendix 1. Our first 3D CNN
We attempted to make our own 3D CNN [model](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/tree/master/our_code).
[Here](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/our_code/human_detection_report.md) you can see report for code.
This code detects with 1.0 accuracy (on our data) that person exists on 3D lidar points or not.

