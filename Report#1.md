# Report 1
# Human Detection in Point Cloud Data 


## Done tasks

- Tried a lot of detection models
- Found datasets with lidar points
- Connect lidar points to rosbag
- Created a ROS environment for checking algorithm work with all visualizations
- Design and train a 3D Convolutional Neural Network (CNN) for human detection.
- Evaluated the trained model's performance.
- Found good floor removal
- Found another good code for human detection

## Datasets

Now we do not have our own dataset because at this moment we do not work with real robot. 

We found a lot of datasets in the internet. Now we mainly use parts of KITTY and nuScenes [datasets](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/tree/master/existing_models/datasets) with appropriate for us images. 

### Rosbag

[Here](https://github.com/domrachev03/LiDAR_Tracking_3D/tree/c6e36c287c771da163eb9b53d4543a5c9c4a0041) we took code for setting lidar points to rosbag. It will be used for comfortable using of points in ROS environment.

## ROS environment

Based on [this](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar/tree/27db548ea51d8faa498c9a2492b172219d6a56fb) project we made a ROS environment for us for visualizing all our data and for checking algorithms.

![GIF](http://wiki.ros.org/multi_object_tracking_lidar?action=AttachFile&do=view&target=multi-object-tracking-lidar-demo.gif)

## Our first 3D CNN

We made a 3D CNN [model](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/tree/master/our_code).
[Here](https://github.com/domrachev03/Real-Life-Human-Detection-with-3D-Lidar-data/blob/master/our_code/human_detection_report.md) you can see report for code.
This code detects with 1.0 accuracy(on our data) that person exists on 3D lidar points or not.

## Floor removal

Found good [floor removal](https://github.com/url-kaist/patchwork-plusplus-ros/tree/6f9b081d68d0c9eddda74d2763de1d3f8e51ac04)

## Another good human detector(with coordinates)

Based on this [project](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar/tree/27db548ea51d8faa498c9a2492b172219d6a56fb) we had done a human detector based on lidar data.

## Conclusion

We had done many work and it is only start of our work:) 
We successfully developed a pipeline for detecting humans in point cloud data using a 3D CNN. The trained model demonstrated high accuracy on the training dataset. However, for real-world applications, it is not applicable for now.
So, we should consider another approach based on human detector scripts which we found.


