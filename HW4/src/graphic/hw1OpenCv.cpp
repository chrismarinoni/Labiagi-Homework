#include "hw1OpenCv.h"


Alberto::Alberto(ros::NodeHandle& nh_) : _nh(nh_){
	//OPPLA'
}

Alberto::~Alberto(){
	//CHE GLI FACCIAMO DISTRUGGERE?
}

cv::Mat Alberto::setFilter1(cv::Mat src_img){
	cv::Mat src_img_f;	
	
	// Convert to floating point
	src_img.convertTo(src_img_f, cv::DataType<float>::type );	
	
	// Map intensities in the range [0,1]
	cv::normalize(src_img_f, src_img_f, 0, 1.0, cv::NORM_MINMAX, cv::DataType<float>::type);
	cv::Mat gaussian_blurred_img, box_blurred_img;
	float gaussian_stddev = 1.0;
	
	// Filter the image with a 3x3 box filter
	cv::boxFilter(src_img_f, box_blurred_img, -1, cv::Size(3,3));
	
	// Filter the image with a gaussian filter with std dev = 1
	cv::GaussianBlur( src_img_f, gaussian_blurred_img, cv::Size(0,0), gaussian_stddev );
	
	// Compute the image derivatives along x and y using Sobel
	cv::Mat dx_img, dy_img;
	cv::Sobel(src_img_f, dx_img, cv::DataType<float>::type, 1, 0, 3);
	cv::Sobel(src_img_f, dy_img, cv::DataType<float>::type, 0, 1, 3);
	cv::Mat gradient_mag_img, abs_dx_img, abs_dy_img, binary_img;
	
	// Compute the gradient magnitude image
	abs_dx_img = cv::abs(dx_img);
	abs_dy_img = cv::abs(dy_img);
	gradient_mag_img = 0.5*(abs_dx_img + abs_dy_img);
	
	// Binarize the image
	cv::threshold ( gradient_mag_img, binary_img, 0.4, 1.0, cv::THRESH_BINARY );
		
	return binary_img;
}

cv::Mat Alberto::setFilter2(cv::Mat src_img){

  cv::Mat src, dst;

  cv::Mat kernel;
  cv::Point anchor;
  double delta;
  int ddepth;
  int kernel_size;

  int c;

  /// Load an image
  src = src_img;

  /// Initialize arguments for the filter
  anchor = cv::Point( -1, -1 );
  delta = 0;
  ddepth = -1;


      /// Update kernel size for a normalized box filter
  kernel_size = 3 + 2*( 10 );
  kernel = cv::Mat::ones( kernel_size, kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);

      /// Apply filter
  cv::filter2D(src, dst, ddepth , kernel, anchor, delta, cv::BORDER_DEFAULT );


  return dst;
}

cv::Mat Alberto::setFilter3(cv::Mat src_img){
	
	int threshold_value = 0;
	int threshold_type = 0;
	int const max_value = 255;
	int const max_type = 4;
	int const max_BINARY_value = 255;

	cv::Mat src_gray, dst;
	
	/// Convert the image to Gray
    cv::cvtColor( src_img, src_gray, CV_BGR2GRAY );
    
    cv::threshold( src_gray, dst, threshold_value, max_BINARY_value,threshold_type );
    
    return dst;
}


