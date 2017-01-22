#include <ros/ros.h>
#include "image_publisher.hpp"

namespace universal_map_server {
  
ImagePublisher::ImagePublisher(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      published_(false)
{
  readParameters();
}

ImagePublisher::~ImagePublisher()
{ 
}

bool ImagePublisher::readParameters()
{
  nodeHandle_.param("file_path", filePath_, std::string("/home/tera/catkin_ws/src/universal_map_server/maps/gaisyuu_for_costmap_edit.png"));
  nodeHandle_.param("image_topic", imageTopic_, std::string("image"));
  nodeHandle_.param("frame_id", frameId_, std::string("map"));
}

void ImagePublisher::imageToTopic()
{
  //cv_image = cv::imread("/home/tera/catkin_ws/src/traversal_layer/maps/mymap_for_costmap.png",CV_LOAD_IMAGE_COLOR);
  cv_image = cv::imread(filePath_, CV_LOAD_IMAGE_COLOR);
  cv::Point2f center(cv_image.cols*0.5+7, cv_image.rows*0.5);
  const cv::Mat affine_matrix = cv::getRotationMatrix2D(center, angle, scale);
  cv::warpAffine(cv_image, rot_cv_image, affine_matrix, cv_image.size());

  origin_x = rot_cv_image.cols*0.5-cv_image.rows*0.5;
  origin_y = 0;
  cut_x = cv_image.rows;
  cut_y = cv_image.rows;
  cv::Rect rect(origin_x, origin_y, cut_y, cut_x);
  cv::Mat cut_image(rot_cv_image, rect);

  cv_bridge::CvImage out_image;
  out_image.encoding = "bgr8";
//  out_image.image = rot_cv_image;
  out_image.image = cut_image;

  out_image.toImageMsg(ros_image);
  ros_image.header.frame_id = frameId_;

  ros::Rate loop_rate(1);
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh("~");
  universal_map_server::ImagePublisher imagePublisher(nh);
  imagePublisher.imageToTopic();
  imagePublisher.pubImage_ = nh.advertise<sensor_msgs::Image>(imagePublisher.imageTopic_, 1, true);

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(10.0);
  ros::Rate loop_rate(1);
  while(ros::Time::now() - start_time < timeout)
  {
    imagePublisher.pubImage_.publish(imagePublisher.ros_image);
    loop_rate.sleep();
  }
}
