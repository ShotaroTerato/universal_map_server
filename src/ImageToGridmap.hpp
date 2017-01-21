#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <string>

namespace universal_map_server {
  
class ImageToGridmap
{
public:
  ImageToGridmap(ros::NodeHandle& nodeHandle);
  virtual ~ImageToGridmap();
  bool readParameters();
  void imageCallback(const sensor_msgs::Image& msg);
  ros::Publisher gridMapPublisher_;
  grid_map::GridMap map_;
  
private:
  ros::NodeHandle& nodeHandle_;
  ros::Subscriber imageSubscriber_;
  std::string imageTopic_;
  double mapLengthX_;
  double resolution_;
  double minHeight_;
  double maxHeight_;
  std::string mapFrameId_;
  
  bool mapInitialized_;
};

}/* namespace */