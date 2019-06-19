#include "ros/ros.h"
#include "bartolo.h"
#include <iostream>


using namespace cv;

std::string path;


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
	
	ros::init(argc, argv, "bartolo_gicp");
	ros::NodeHandle nh("~");
	int num_pcd;
	if(argc > 2){
		num_pcd = atoi(argv[1]);
		path = argv[2];
		
	}
	
	
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> ("./"+path+"/res_cloud_1.pcd", *cloud_out);
    
    std::cout << *cloud_out << std::endl;
    
    //visualizeNow(cloud_out);
    
    std::cout << ">>res_cloud_1.pcd has been opened and visualized" << std::endl;


   // pcl::PCLPointCloud2 cloud_blob;
   
   Eigen::Matrix4f globalTransform = Eigen::Matrix4f::Identity();

	
	for(int i = 2; i <= num_pcd; i++){
		
		// create the object implementing ICP algorithm
		pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBNormal,  pcl::PointXYZRGBNormal>gicp;
		
		pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> ("./"+path+"/res_cloud_"+std::to_string(i)+".pcd", *cloud_in);
		// set the input point cloud to align  
		
		//visualizeNow(cloud_in);
		
		gicp.setInputSource(cloud_in);
		
		std::cout << ">> Applying GICP with res_cloud_" << std::to_string(i) << ".pcd" << std::endl;

		
		// set the input reference point cloud  
		gicp.setInputTarget(cloud_out);
		
		// compute the point cloud registration  
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZRGBNormal>);  
		gicp.align(*cloud_trans);
		
		Eigen::Matrix4f alignmentTransform = gicp.getFinalTransformation();
		globalTransform *= alignmentTransform;
		
		// print if it the algorithm converged and its fitness score  
		std::cout << "has converged:" << gicp.hasConverged() << " score:"<< gicp.getFitnessScore() << std::endl;
		
		// print the output transformation
		std::cout << globalTransform <<std::endl;

		  
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		
		pcl::transformPointCloud (*cloud_in, *cloud_transformed, gicp.getFinalTransformation());
	 	
	        			
	 	*cloud_out += *cloud_transformed;
	 	
	 	std::cout << *cloud_trans << std::endl;
	 	

		std::cout << "Dimensioni matrix out: " << cloud_out->size() << " || Dimensioni matrix transformed: " <<
			cloud_trans->size() << " || Dimensioni matrix source: " << cloud_in->size() << std::endl;
		
		
		visualizeNow(cloud_out);

		
		
	}
	

	if ( pcl::io::savePCDFileASCII( "./"+path+"/res_cloud_result.pcd", *cloud_out) < 0) {
				std::cout << "Error saving model cloud." << std::endl;
				return (-1);  
    }
	
	
	pcl::PassThrough<pcl::PointXYZRGBNormal> pass_through;  
	pass_through.setInputCloud(cloud_out);  
	pass_through.setFilterLimits(0.0, 0.7);  
	pass_through.setFilterFieldName("z");  
	pass_through.filter(*cloud_out);
	
	visualizeNow(cloud_out);

	
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_voxelized(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> voxel_grid;  
	voxel_grid.setInputCloud(cloud_out);  
	voxel_grid.setLeafSize(0.01, 0.01, 0.01);  
	voxel_grid.filter(*cloud_voxelized);
	
	visualizeNow(cloud_out);
	
}

