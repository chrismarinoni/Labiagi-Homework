#include "bartolo.h"

using namespace sensor_msgs;
using namespace message_filters;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
int i = 0;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth2PointCloud(const cv::Mat &depth_img, const cv::Mat &rgb_img, const Eigen::Matrix3f &camera_matrix, float min_depth = 0.01f, float max_depth = 10.0f)
{
  // Create a new cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pc_ptr->points.resize(depth_img.total());
  Eigen::Matrix4f t_mat;
  t_mat.setIdentity();
  t_mat.block<3, 3>(0, 0) = camera_matrix.inverse();

  int i_pt = 0;
  for (int y = 0; y < depth_img.rows; y++)
  {
    const float *depth = depth_img.ptr<float>(y);
    const uchar *bgr = rgb_img.ptr<uchar>(y);
    uchar r, g, b;
    for (int x = 0; x < depth_img.cols; x++, depth++, bgr += 3)
    {
      float d = *depth;
      if (d >= min_depth && d <= max_depth)
      {
        Eigen::Vector4f point = t_mat * Eigen::Vector4f(x * d, y * d, d, 1.0);

        b = bgr[0];
        g = bgr[1];
        r = bgr[2];

        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 |
                        static_cast<uint32_t>(b));

        pc_ptr->points[i_pt].x = point(0);
        pc_ptr->points[i_pt].y = point(1);
        pc_ptr->points[i_pt].z = point(2);

        pc_ptr->points[i_pt++].rgb = *reinterpret_cast<float *>(&rgb);
      }
    }
  }

  pc_ptr->points.resize(i_pt);
  pc_ptr->width = (int)pc_ptr->points.size();
  pc_ptr->height = 1;

  return pc_ptr;
}


void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception &ex)
  {
    ROS_ERROR("cv_bridge exception: %s", ex.what());
    exit(-1);
  }
  cv::imshow("DEPTH", cv_ptr->image);
  cv::waitKey(30);
}

void rgbCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &ex)
  {
    ROS_ERROR("cv_bridge exception: %s", ex.what());
    exit(-1);
  }
  cv::imshow("RGB", cv_ptr->image);
  cv::waitKey(30);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "rosbag viewer");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rosbag viewer");
  viewer->initCameraParameters();
  //viewer->setCameraPosition(0, -1, 0,    0, -1, 0,   0, 0, 1);
  return (viewer);
}

void callback(const ImageConstPtr &msg1, const ImageConstPtr &msg2)
{
  cv::Mat depth_img;
  cv::Mat input_depth = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  cv::Mat rgb_img = cv_bridge::toCvShare(msg2, sensor_msgs::image_encodings::BGR8)->image;	
  double depth_scale = 0.001;
  input_depth.convertTo(depth_img, CV_32F, depth_scale);
  float fx = 512, fy = 512,
        cx = 320, cy = 240;

  Eigen::Matrix3f camera_matrix;
  camera_matrix << fx, 0.0f, cx,
      0.0f, fy, cy,
      0.0f, 0.0f, 1.0f;

  // Generate and visualize the point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = depth2PointCloud(depth_img, rgb_img, camera_matrix);
  if(i == 0){
    viewer = rgbVis(cloud_ptr);
    i = 1;
  } else {
    viewer->updatePointCloud(cloud_ptr, "rosbag viewer");
    viewer->spinOnce();
  }
}

int main(int argc, char **argv)
{
  ROS_INFO_STREAM("Starting PCL Node");
  ros::init(argc, argv, "rosbag_subscriber");
  ros::NodeHandle nh;
  message_filters::Subscriber<Image> depth_sub(nh, "/camera/depth_registered/hw_registered/image_rect_raw", 1);
  message_filters::Subscriber<Image> rgb_sub(nh, "/camera/rgb/image_rect_color", 1);
  typedef sync_policies::ApproximateTime<Image, Image> syncPolicy;
  Synchronizer<syncPolicy> sync(syncPolicy(10), depth_sub, rgb_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
  return 0;
}


