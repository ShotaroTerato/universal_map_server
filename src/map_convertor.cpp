#include "map_convertor.hpp"

namespace map_convertor {

MapConvertor::MapConvertor(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  //gridMapSubscriber_ = nodeHandle_.subscribe("/grid_map", 1, &MapConvertor::mapCallback, this);
  //occMapPublisher_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("/occ_map", 1, true);
  //imagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/occ_image", 1, true);
  //gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/merge_map", 1, true);
}

MapConvertor::~MapConvertor()
{
}

void MapConvertor::readParameters(const std::string& robot_name)
{
  nodeHandle_.param(robot_name+"/max_traversable_step_height", occThreshold_, 0.02);
}

void MapConvertor::mapCallback(const grid_map_msgs::GridMap& grid_map_msgs)
{
  grid_map::GridMapRosConverter::fromMessage(grid_map_msgs, grid_map_);
} 

void MapConvertor::thresholding(grid_map::GridMap& in_grid_map, const std::string& layer)
{ 
  minHeight_ = 0.0; //initial value
  maxHeight_ = 0.0; //initial value
  for(grid_map::GridMapIterator iterator(in_grid_map); !iterator.isPastEnd(); ++iterator){
    if(in_grid_map.at(layer, *iterator) < minHeight_){
      minHeight_ = in_grid_map.at(layer, *iterator);
    }else if(in_grid_map.at(layer, *iterator) > maxHeight_){
      maxHeight_ = in_grid_map.at(layer, *iterator);
    }
  }
  
  double threshold = maxHeight_ - minHeight_ - occThreshold_;
  for(grid_map::GridMapIterator iterator(in_grid_map); !iterator.isPastEnd(); ++iterator){
    if(in_grid_map.at(layer, *iterator) <= (threshold)){
      in_grid_map.at(layer, *iterator) = 0.0;
    }
  }
}

void MapConvertor::mergeLayer(grid_map::GridMap& in_grid_map, const std::string& origin_layer, const std::string& add_layer)
{
  for(grid_map::GridMapIterator iterator(in_grid_map); !iterator.isPastEnd(); ++iterator){
    if(in_grid_map.at(origin_layer, *iterator) > in_grid_map.at(add_layer, *iterator)){
      in_grid_map.at(origin_layer, *iterator) = in_grid_map.at(add_layer, *iterator);
    }
  }
}

void MapConvertor::toCvImage(const grid_map::GridMap& in_grid_map, const std::string& to_image_layer)
{
  grid_map::GridMapRosConverter::toCvImage(in_grid_map, to_image_layer, "bgr8", cv_image_);
  cv_image_.toImageMsg(ros_image_);
}

}//namespace