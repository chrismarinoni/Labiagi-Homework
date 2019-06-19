#pragma once
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>  
// #include <pcl/feature/pcd_feature.h>  
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <pcl/visualization/pcl_visualizer.h>
 #include <pcl/visualization/cloud_viewer.h>
 #include <pcl/visualization/point_cloud_handlers.h>
 #include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/gicp.h>
#include <message_filters/subscriber.h>  
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include <pcl/filters/passthrough.h>


class Bartolo{
public:
  Bartolo(ros::NodeHandle& nh_);
  ~Bartolo();
    
protected:
  ros::NodeHandle& _nh;
};
