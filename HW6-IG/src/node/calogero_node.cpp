#include "ros/ros.h"
#include "calogero.h"

using namespace std;

 /** Global variables */
 string frontal_face_cascade_name = "haarcascade_frontalface_alt.xml";
 string eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
 string profile_face_cascade_name = "haarcascade_profileface.xml";
 cv::CascadeClassifier frontal_face_cascade;
 cv::CascadeClassifier profile_face_cascade;
 cv::CascadeClassifier eyes_cascade;

 //string window_name = "Capture - Face detection";
 cv::RNG rng(12345);


int main(int argc, char** argv){

	ros::init(argc, argv, "calogero_node");
	
	//CvCapture* capture;
    cv::Mat frame;

	ros::NodeHandle nh("~");
  
//	ros::Publisher alberto_pub = nh.advertise<std_msgs::Image>("alberto_sending_image", 1000);
//	ros::Rate loop_rate(10);

	//image_transport::ImageTransport it(nh);
	//image_transport::Publisher pub = it.advertise("camera/image", 1);
	
	if( !frontal_face_cascade.load( frontal_face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
	if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
	if( !profile_face_cascade.load( profile_face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };

	
	//-- 2. Read the video stream
	cv::VideoCapture cap = cv::VideoCapture("/dev/video0");
	if(!cap.isOpened()) return 1;
	//capture = cvCaptureFromCAM( -1 );
	
	cv::Size recoFace_size = cv::Size(0,0);
	
	ros::Rate loop_rate(5);
	while( nh.ok() )
	{
		Calogero calogero(nh);

		cap >> frame;
		//frame = cvQueryFrame( capture );
			//-- 3. Apply the classifier to the frame
		if( !frame.empty() )
		{ recoFace_size = calogero.detectAndDisplay( frame , frontal_face_cascade, profile_face_cascade, eyes_cascade, recoFace_size); }
		else
		{ printf(" --(!) No captured frame -- Break!"); break; }
		int c = cv::waitKey(10);
		if( (char)c == 'c' ) { break; }
		cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
 }
