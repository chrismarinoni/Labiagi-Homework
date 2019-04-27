#include "ros/ros.h"
#include "hw1OpenCv.h"

using namespace std;
using namespace cv;


int main(int argc, char** argv){

	ros::init(argc, argv, "alberto_talker");

	ros::NodeHandle nh("~");
  
//	ros::Publisher alberto_pub = nh.advertise<std_msgs::Image>("alberto_sending_image", 1000);
//	ros::Rate loop_rate(10);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  cv::VideoCapture cap = cv::VideoCapture(0);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return 1;
  cv::Mat frame, frame2;
  sensor_msgs::ImagePtr msg, msg_output;

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      
      imshow("Frame", frame);
      
      if(argc > 1){
		  Alberto alberto(nh);
		  if(atoi(argv[1]) == 1)
			frame2 = alberto.setFilter1(frame);
          else if(atoi(argv[1]) == 2)
            frame2 = alberto.setFilter2(frame);
          else if(atoi(argv[1]) == 3)
            frame2 = alberto.setFilter3(frame);
          else
			std::cerr << argv[1] << std::endl;
      }
		
	  cv::imshow("func", frame2);
	
		
	  msg_output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame2).toImageMsg();
		
	  imshow("FrameMod", frame2);


      pub.publish(msg_output);
      
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
 
 
}
