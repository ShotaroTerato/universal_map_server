#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

namespace map_convertor {
  
class MapConvertor
{
public:
  MapConvertor(ros::NodeHandle& nodeHandle);
  virtual ~MapConvertor();
  void mapCallback(const grid_map_msgs::GridMap& grid_map_msgs);

private:
  ros::NodeHandle& nodeHandle_;
  ros::Subscriber gridMapSubscriber_;
  ros::Publisher gridMapPublisher_;
  ros::Publisher imagePublisher_;
  nav_msgs::OccupancyGrid occ_map_;
  nav_msgs::GridCells grid_cell_;
  sensor_msgs::PointCloud2 layer_cloud_;
  grid_map::GridMap grid_map_;
  cv_bridge::CvImage cv_image_;
  sensor_msgs::Image ros_image_;
};
}//namespace