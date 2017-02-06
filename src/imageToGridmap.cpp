#include "ImageToGridmap.hpp"

namespace universal_map_server {
  
ImageToGridmap::ImageToGridmap(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      mapInitialized_(false)
{
  readParameters();
}

ImageToGridmap::~ImageToGridmap()
{
}

bool ImageToGridmap::readParameters()
{
  nodeHandle_.param("resolution", resolution_, 0.05);
  nodeHandle_.param("min_height", minHeight_, 0.0); //Black
  nodeHandle_.param("max_height", maxHeight_, 0.5); //White
  nodeHandle_.param("frame_id", frameId_, std::string("map"));
  return true;
}

void ImageToGridmap::imageToMsg()
{
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
  out_image.image = cut_image;

  out_image.toImageMsg(ros_image_);
  ros_image_.header.frame_id = frameId_;
}

void ImageToGridmap::addLayer(const sensor_msgs::Image& msg, const std::string& layerName)
{
  if (!mapInitialized_) {
    map_.setBasicLayers({layerName});
    grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, map_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
            map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
    mapInitialized_ = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(msg, layerName, map_, minHeight_, maxHeight_);

}

} // namespace