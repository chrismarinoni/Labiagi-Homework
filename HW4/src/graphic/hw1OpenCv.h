#pragma once
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sstream"
#include "std_msgs/String.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

class Alberto{
public:
  Alberto(ros::NodeHandle& nh_);
  ~Alberto();
  
  cv::Mat setFilter1(cv::Mat src_img);
  cv::Mat setFilter2(cv::Mat src_img);
  cv::Mat setFilter3(cv::Mat src_img);
  
    
protected:
  ros::NodeHandle& _nh;
};
