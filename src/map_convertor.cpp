#include "map_convertor.hpp"

namespace map_convertor {

MapConvertor::MapConvertor(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  gridMapSubscriber_ = nodeHandle_.subscribe("/grid_map", 1, &MapConvertor::mapCallback, this);
  //occMapPublisher_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("/occ_map", 1, true);
  imagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/occ_image", 1, true);
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/merge_map", 1, true);
}

MapConvertor::~MapConvertor()
{
}

void MapConvertor::mapCallback(const grid_map_msgs::GridMap& grid_map_msgs)
{
  grid_map::GridMapRosConverter::fromMessage(grid_map_msgs, grid_map_);
  grid_map_.add("merge_layer", (grid_map_.get("layer1") + grid_map_.get("layer2"))/2);
  
  //float occThreshold = 0.002;
  for(grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator){
    //ROS_INFO("it: %f", grid_map_.at("merge_layer", *iterator));
    //conditions for each layer
    if(grid_map_.at("layer1", *iterator) <= (0.5-0.02)){
      grid_map_.at("merge_layer", *iterator) = 0.0;
    }
  }
  
  grid_map::GridMapRosConverter::toCvImage(grid_map_, "merge_layer", "bgr8", cv_image_);
  cv_image_.toImageMsg(ros_image_);
  
  //Publish as grid map
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(grid_map_, mapMessage);
  gridMapPublisher_.publish(mapMessage);
  
  imagePublisher_.publish(ros_image_);
} 

}//namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_convertor");
  ros::NodeHandle nh("~");
  map_convertor::MapConvertor mapConvertor(nh);
  
  ros::spin();
  return 0;
}