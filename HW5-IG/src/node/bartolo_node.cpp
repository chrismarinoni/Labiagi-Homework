#include "ros/ros.h"
#include "bartolo.h"
#include <iostream>

using namespace cv;


pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth2PointCloud(const cv::Mat &depth_img, 
														const cv::Mat &rgb_img, 
														const Eigen::Matrix3f &camera_matrix, 
														float min_depth = 0.01f, 
														float max_depth = 10.0f)
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



pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr createXYZRBGNormal (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,
															pcl::PointCloud<pcl::Normal>::Ptr cloud_normal){
																
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_outF (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				
	std::cout << "> I'm preparing to merge information coming from two different types of cloud. Input data: cloud_rgb " << cloud_rgb->size() 
			  << " - cloud_normal " << cloud_normal->size() << std::endl;
		
	cloud_outF->points.resize ( cloud_rgb->size() );
	

	int i;
		
	for( i = 0; i < cloud_rgb->size(); i++)
	{

		cloud_outF->points[i].x = cloud_rgb->points[i].x;
		cloud_outF->points[i].y = cloud_rgb->points[i].y;
		cloud_outF->points[i].z = cloud_rgb->points[i].z;

		cloud_outF->points[i].curvature = cloud_normal->points[i].curvature; 	

			
		cloud_outF->points[i].rgb = cloud_rgb->points[i].rgb;

	}
	
	std::cout << "> Merge DONE! Merged cloud dimension: " << cloud_outF->size() << std::endl;
	
	cloud_outF->points.resize(i+1);
	cloud_outF->width = (int)cloud_outF->points.size();
	cloud_outF->height = 1;
	
	return cloud_outF;															
																
}




int main(int argc, char** argv){
	
	ros::init(argc, argv, "bartolo_pcl");
	
	ros::NodeHandle nh("~");
	
	int i = 1;
	for(; i <99; i++) {
		Mat imageC, imageDepthMeters;
		imageC = imread("./desk_1/desk_1_" + std::to_string(i)	 +  ".png", IMREAD_COLOR);
		Mat imageDepth = imread("./desk_1/desk_1_" + std::to_string(i) + "_depth.png", IMREAD_ANYDEPTH);
		
		std::cout << ">> Loading images desk_1_" << std::to_string(i) << "_depth.png" << std::endl;


			
			imageDepth.convertTo(imageDepthMeters, CV_32F, 0.001);
			
			if(!imageDepthMeters.data){
				std::cerr << "No depth data! I can't execute your code." << std::endl;
				exit(EXIT_FAILURE);
			}
			
			// Parametro di scaling kinect
			double depth_scale = 0.001;  

			// Intrisincs parameters 
			float fx = 512, fy = 512, 
					cx = 320, cy = 240;
					
			Eigen::Matrix3f camera_matrix;
			camera_matrix<< fx,   0.0f, cx,
							0.0f, fy,   cy,
							0.0f, 0.0f, 1.0f;
			
			
			// Show both images
			cv::Mat show_depth;
			cv::normalize(imageDepthMeters, show_depth, 0, 255, cv::NORM_MINMAX);
			//cv::imshow("depth", show_depth );
			//cv::imshow("rgb", rgb_img );	

			pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr = depth2PointCloud( imageDepthMeters, imageC, camera_matrix );

			
			// Create the normal estimation class, and pass the input dataset to it
			  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
			  ne.setInputCloud (cloud_ptr);

			  // Create an empty kdtree representation, and pass it to the normal estimation object.
			  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
			  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
			  ne.setSearchMethod (tree);

			  // Output datasets
			  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

			  // Use all neighbors in a sphere of radius 3cm
			  ne.setRadiusSearch (0.03);

			  // Compute the features
			  ne.compute (*cloud_normals);

		  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		  
		  cloud = createXYZRBGNormal(cloud_ptr, cloud_normals);
		  

		  
		  if ( pcl::io::savePCDFileASCII( "./desk_1/res_cloud_"+std::to_string(i)+".pcd", *cloud) < 0) {
				std::cout << "Error saving model cloud." << std::endl;
				return (-1);
		  }
		  
		  std::cout << "> Saved related pcd" << std::endl;


		
	}
	
}
