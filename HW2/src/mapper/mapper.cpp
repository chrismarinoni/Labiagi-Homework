#include "mapper.h"
#include "math.h"
#include <iostream>

/* 
 *******
 * All the other homework are available on the GitHub repository
 * https://github.com/chrismarinoni/Homework-Labiagi
 * 
 * Note: Use this code only if you are in a deadlock.
 *       Always try to do the homework by yourself ;)
 *******
*/

LaserMapper::LaserMapper(ros::NodeHandle& nh_) : _nh(nh_){
  _listener = new tf::TransformListener;
  _odom_frame_id = "";
  _laser_topic = "";
}

LaserMapper::~LaserMapper(){
  delete _listener;
}

void LaserMapper::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_){
  tf::StampedTransform transform;
  // Get laser transform in odom frame using the tf listener
  
  if(_listener->canTransform(_odom_frame_id, msg_->header.frame_id, msg_->header.stamp, NULL)){
	_listener->lookupTransform(_odom_frame_id, msg_->header.frame_id, msg_->header.stamp, transform); 
  }
  
  //

  Eigen::Isometry2f T = convertPose2D(transform);
  // Extract points from raw laser scan and paint them on canvas

  // for (something)
  //
  //  Eigen::Vector2f p

  //
  //  Eigen::Vector2f transformed_point = T * p
  //
  //
  float angle;
  Eigen::Vector2f transformed_point;
  for(int i = 0; i < msg_->ranges.size(); i++) {
    Eigen::Vector2f p;
	angle = msg_->angle_min + i * msg_->angle_increment;
	p[0] = msg_->ranges[i]*cos(angle);
	p[1] = msg_->ranges[i]*sin(angle);
	transformed_point = T * p;
	std::cerr<<transformed_point<<std::endl;
	_canvas->colorPoint(transformed_point);
  }
  _canvas->show();
}

void LaserMapper::subscribe(){

  _laser_sub = _nh.subscribe(_laser_topic,10,&LaserMapper::laserCallback,this);
  _odom_sub = _nh.subscribe(_odom_frame_id,10,&LaserMapper::odomCallback,this);
}

void LaserMapper::odomCallback(const nav_msgs::Odometry::ConstPtr& msg_){
	const cv::Vec3b & color = cv::Vec3b(0,0,255);
	Eigen::Vector2f p;
	p[0] = msg_->pose.pose.position.x;
	p[1] = msg_->pose.pose.position.y;
	std::cerr<<p<<std::endl;
	_canvas->colorPoint(p, color);
	_canvas->show();
}

