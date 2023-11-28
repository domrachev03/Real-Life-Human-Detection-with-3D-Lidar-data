#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher image1_pub;
ros::Publisher image2_pub;
ros::Publisher pcl_pub;

void callback(const ImageConstPtr& image1, const ImageConstPtr& image2, const PointCloud2ConstPtr& pcl)
{
  image1_pub.publish(*image1);
  image2_pub.publish(*image2);
  pcl_pub.publish(*pcl);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filtered");

  ros::NodeHandle nh("~");
  image1_pub = nh.advertise<sensor_msgs::Image>(
        "fisheye1/image_raw", 1);
  image2_pub = nh.advertise<sensor_msgs::Image>(
        "fisheye2/image_raw", 1);
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(
        "velodyne_points", 1);
               
  message_filters::Subscriber<Image> image1_sub(nh, "/front_camera/fisheye1/image_raw", 1);
  message_filters::Subscriber<Image> image2_sub(nh, "/front_camera/fisheye2/image_raw", 1);
  message_filters::Subscriber<PointCloud2> pcl_sub(nh, "/velodyne_points", 1);

  typedef sync_policies::ApproximateTime<Image, Image, PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub, pcl_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();
  return 0;
}