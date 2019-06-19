#include "ros/ros.h"
#include "bartolo.h"
#include <iostream>

using namespace cv;


void visualizeNow(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out){
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
				
	std::cout << "> I'm preparing all the stuff to visualize" << std::endl;
		
	cloud_rgb->points.resize ( cloud_out->size() );
		
	for(int i = 0; i < cloud_out->size(); i++)
	{
		
		cloud_rgb->points[i].x = cloud_out->points[i].x;
		cloud_rgb->points[i].y = cloud_out->points[i].y;
		cloud_rgb->points[i].z = cloud_out->points[i].z;
		
		cloud_rgb->points[i].rgb = cloud_out->points[i].rgb;
		

	}
		
	std::cout << "> Ready to visualize" << std::endl;
	
	pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud_rgb, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
		       									3, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
		  
		  
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce ( 1 );
	}
	viewer->removePointCloud("sample cloud");
	
	viewer->close();
	
}



int main(int argc, char** argv){
	
	ros::init(argc, argv, "bartolo_visualizer");
	
	ros::NodeHandle nh("~");
	
	std::string path;
	if(argc > 1)
		path = argv[1];
	else
		ROS_ERROR("Inserisci il nome del file da prelevare da questa directory ed, eventualmente, la subdirectory");


	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_to_vis (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> ("./"+path, *cloud_to_vis);
	
	visualizeNow(cloud_to_vis);
	
}
