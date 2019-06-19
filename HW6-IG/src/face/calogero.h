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
#include <cstring>
#include <iostream>

#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"


class Calogero{
public:
  Calogero(ros::NodeHandle& nh_);
  ~Calogero();
  
  cv::Size detectAndDisplay( cv::Mat frame, 
					     cv::CascadeClassifier frontal_face_cascade,
					     cv::CascadeClassifier profile_face_cascade,
					     cv::CascadeClassifier eyes_cascade,
					     cv::Size recoFace_size);

    
protected:
  ros::NodeHandle& _nh;
};
