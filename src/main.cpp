#include <ros/ros.h>
#include "ImageToGridmap.hpp"
#include "image_publisher.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "universal_map_server");
  ros::NodeHandle nh("~");
  grid_map::GridMap grid_map;
  universal_map_server::ImageToGridmap imageToGridmap(nh);
  imageToGridmap.readParameters();
  imageToGridmap.imageToMsg();
  imageToGridmap.addLayer(imageToGridmap.ros_image_);

  ros::spin();
  return 0;
}