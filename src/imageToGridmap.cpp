#include "ImageToGridmap.hpp"

namespace universal_map_server {
  
ImageToGridmap::ImageToGridmap(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
//      map_(grid_map::GridMap({"elevation"})),
//      mapInitialized_(false)
{
  readParameters();
  map_.setBasicLayers({layerName_});
  imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &ImageToGridmap::imageCallback, this);
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
}

ImageToGridmap::~ImageToGridmap()
{
}

bool ImageToGridmap::readParameters()
{
  nodeHandle_.param("image_topic", imageTopic_, std::string("/image_publisher/image"));
  nodeHandle_.param("layer_name", layerName_, std::string("layer1"));
  nodeHandle_.param("resolution", resolution_, 0.05);
  nodeHandle_.param("min_height", minHeight_, 0.0); //Black
  nodeHandle_.param("max_height", maxHeight_, 1.0); //White
  return true;
}

void ImageToGridmap::imageCallback(const sensor_msgs::Image& msg)
{
  if (!mapInitialized_) {
    grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, map_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
            map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
    mapInitialized_ = true;
  }
  //grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", map_, minHeight_, maxHeight_);
  //grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "elevation", map_);
  grid_map::GridMapRosConverter::addColorLayerFromImage(msg, layerName_, map_);

  // Publish as grid map.
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(map_, mapMessage);
  gridMapPublisher_.publish(mapMessage);
}

} // namespace