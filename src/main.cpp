#include <ros/ros.h>
#include "ImageToGridmap.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "universal_map_server");
  ros::NodeHandle nh("~");
  universal_map_server::ImageToGridmap imageToGridmap(nh);
  
  imageToGridmap.gridMapPublisher_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  // Publish as grid map.
//  grid_map_msgs::GridMap mapMessage;
//  grid_map::GridMapRosConverter::toMessage(imageToGridmap.map_, mapMessage);
//  imageToGridmap.gridMapPublisher_.publish(mapMessage);
  ros::spin();
  return 0;
}