# Report 1
# Human Detection in Point Cloud Data 

## Link to [github](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data.git)

> Note. Unfortunately, we failed to finish a proper report before deadline. Please, consider the report from the github repo. It provides great visualizations etc.

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

## ROS environment
Based on [this](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar/tree/27db548ea51d8faa498c9a2492b172219d6a56fb) project we made a ROS environment for us for visualizing all our data and for checking algorithms.

## Our first 3D CNN
We made a 3D CNN [model](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/tree/master/our_code).
[Here](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/our_code/human_detection_report.md) you can see report for code.
This code detects with 1.0 accuracy(on our data) that person exists on 3D lidar points or not.

## Floor removal

Found good [floor removal](https://github.com/url-kaist/patchwork-plusplus-ros/tree/6f9b081d68d0c9eddda74d2763de1d3f8e51ac04)

## Another good human detector(with coordinates)

Based on this [project](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar/tree/27db548ea51d8faa498c9a2492b172219d6a56fb) we had done a human detector based on lidar data.

![GIF](http://wiki.ros.org/multi_object_tracking_lidar?action=AttachFile&do=get&target=multi-object-tracking-lidar-demo.gif)

## Conclusion

We had done many work and it is only start of our work:) 
We successfully developed a pipeline for detecting humans in point cloud data using a 3D CNN. The trained model demonstrated high accuracy on the training dataset. However, for real-world applications, it is not applicable for now.
So, we should consider another approach based on human detector scripts which we found.


