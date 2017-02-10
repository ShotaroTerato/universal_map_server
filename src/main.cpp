#include <ros/ros.h>
#include "ImageToGridmap.hpp"
#include "map_convertor.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "universal_map_server");
  ros::NodeHandle nh("~");
  ros::Publisher gridMapPublisher = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
  ros::Publisher imagePublisher = nh.advertise<sensor_msgs::Image>("/occ_image", 1, true);
  
  //first map and layer
  universal_map_server::ImageToGridmap imageToGridmap(nh);
  imageToGridmap.readParameters("/tsudanuma_map_1");
  imageToGridmap.imageToMsg();
  imageToGridmap.addLayer(imageToGridmap.ros_image_, imageToGridmap.layerName_);
  
  //second layer into first map
  universal_map_server::ImageToGridmap imageToGridmap_2(nh);
  imageToGridmap_2.readParameters("/tsudanuma_map_2");
  imageToGridmap_2.imageToMsg();
  imageToGridmap.addLayer(imageToGridmap_2.ros_image_, imageToGridmap_2.layerName_);
  
  
  //merge all layer and convert occ map image
  map_convertor::MapConvertor mapConvertor(nh);
  map_convertor::MapConvertor mapConvertor_2(nh);
  mapConvertor.thresholding(imageToGridmap.map_, "layer1");
  mapConvertor_2.thresholding(imageToGridmap.map_, "layer2");
  mapConvertor.mergeLayer(imageToGridmap.map_, "layer1", "layer2");
  mapConvertor.toCvImage(imageToGridmap.map_, "layer1");
  
  // Publish grid map and image
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(imageToGridmap.map_, mapMessage);
  gridMapPublisher.publish(mapMessage);
  
  imagePublisher.publish(mapConvertor.ros_image_);
  ros::spin();
  return 0;
}