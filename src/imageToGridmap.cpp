#include "ImageToGridmap.hpp"

namespace universal_map_server {
  
ImageToGridmap::ImageToGridmap(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
//      map_(grid_map::GridMap({"elevation"})),
      mapInitialized_(false)
{
  readParameters();
  map_.setBasicLayers({layerName_});
  //imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &ImageToGridmap::imageCallback, this);
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
  nodeHandle_.param("file_path", filePath_, std::string("/home/tera/catkin_ws/src/universal_map_server/maps/tsudanuma2_0to05.png"));
  nodeHandle_.param("frame_id", frameId_, std::string("map"));
  return true;
}

void ImageToGridmap::imageToMsg()
{
  //cv_image = cv::imread("/home/tera/catkin_ws/src/traversal_layer/maps/mymap_for_costmap.png",CV_LOAD_IMAGE_COLOR);
  cv_image_ = cv::imread(filePath_, CV_LOAD_IMAGE_COLOR);
  cv::Point2f center(cv_image_.cols*0.5+7, cv_image_.rows*0.5);
  const cv::Mat affine_matrix = cv::getRotationMatrix2D(center, angle, scale);
  cv::warpAffine(cv_image_, rot_cv_image_, affine_matrix, cv_image_.size());

  origin_x = rot_cv_image_.cols*0.5-cv_image_.rows*0.5;
  origin_y = 0;
  cut_x = cv_image_.rows;
  cut_y = cv_image_.rows;
  cv::Rect rect(origin_x, origin_y, cut_y, cut_x);
  cv::Mat cut_image(rot_cv_image_, rect);

  cv_bridge::CvImage out_image;
  out_image.encoding = "bgr8";
//  out_image.image = rot_cv_image_;
  out_image.image = cut_image;

  out_image.toImageMsg(ros_image_);
  ros_image_.header.frame_id = frameId_;
}

void ImageToGridmap::addLayer(const sensor_msgs::Image& msg)
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