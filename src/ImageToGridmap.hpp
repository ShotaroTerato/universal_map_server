#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <string>

namespace universal_map_server {
  
class ImageToGridmap
{
public:
  ImageToGridmap(ros::NodeHandle& nodeHandle);
  virtual ~ImageToGridmap();
  bool readParameters();
  void imageToMsg();
  void addLayer(const sensor_msgs::Image& msg);
  ros::Publisher gridMapPublisher_;
  grid_map::GridMap map_;
  sensor_msgs::Image image_;
  double resolution_;
  sensor_msgs::Image ros_image_;
  
private:
  ros::NodeHandle& nodeHandle_;
  ros::Subscriber imageSubscriber_;
  std::string imageTopic_;
  std::string layerName_;
  double mapLengthX_;
  double minHeight_;
  double maxHeight_;
  
  cv::Mat cv_image_;
  float angle = 90, scale = 1.0;
  cv::Mat rot_cv_image_;
  int origin_x, origin_y, cut_x, cut_y;
  bool published_;
  std::string filePath_;
  std::string frameId_; 
  bool mapInitialized_;
};

}/* namespace */