#include <ros/ros.h>
#include "ImageToGridmap.hpp"
#include "image_publisher.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "universal_map_server");
  ros::NodeHandle nh("~");
  ros::Publisher gridMapPublisher = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
  
  //first map and layer
  universal_map_server::ImageToGridmap imageToGridmap(nh);
  imageToGridmap.filePath_ = std::string("/home/tera/catkin_ws/src/universal_map_server/maps/tsudanuma2_0to05.png");
  imageToGridmap.layerName_ = std::string("layer1");
  imageToGridmap.imageToMsg();
  imageToGridmap.addLayer(imageToGridmap.ros_image_, imageToGridmap.layerName_);
  
  //second layer into first map
  universal_map_server::ImageToGridmap imageToGridmap_2(nh);
  imageToGridmap_2.filePath_ = std::string("/home/tera/catkin_ws/src/universal_map_server/maps/tsudanuma.png");
  imageToGridmap_2.layerName_ = std::string("layer2");
  imageToGridmap_2.imageToMsg();
  imageToGridmap.addLayer(imageToGridmap_2.ros_image_, imageToGridmap_2.layerName_);
  
  // Publish as grid map.
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(imageToGridmap.map_, mapMessage);
  gridMapPublisher.publish(mapMessage);
  
  ros::spin();
  return 0;
}