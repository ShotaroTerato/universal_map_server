#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

namespace universal_map_server {

class ImagePublisher
{
public:
  ImagePublisher(ros::NodeHandle& nodeHandle);
  virtual ~ImagePublisher();
  bool readParameters();
  void imageToTopic();
  ros::Publisher pubImage_;
  std::string imageTopic_;
  sensor_msgs::Image ros_image;

private:
  ros::NodeHandle& nodeHandle_;
  cv::Mat cv_image;
  float angle = 90, scale = 1.0;
  cv::Mat rot_cv_image;
  int origin_x, origin_y, cut_x, cut_y;
  bool published_;
  std::string filePath_;
  std::string frameId_;
}; 
}