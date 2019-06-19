	#include "calogero.h"

Calogero::Calogero(ros::NodeHandle& nh_) : _nh(nh_){
	//OPPLA'
}

Calogero::~Calogero(){
	//CHE GLI FACCIAMO DISTRUGGERE?
}

cv::Size Calogero::detectAndDisplay( cv::Mat frame, 
					   cv::CascadeClassifier frontal_face_cascade,
					   cv::CascadeClassifier profile_face_cascade,
					   cv::CascadeClassifier eyes_cascade,
					   cv::Size recoFace_size)
{
  std::vector<cv::Rect> faces;
  std::vector<cv::Rect> profile;

  cv::Mat frame_gray;

  cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
  cv::equalizeHist( frame_gray, frame_gray );

  //-- Detect faces
  frontal_face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0| CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
  
  

  for( size_t i = 0; i < faces.size(); i++ )
  {
	   
    
    //cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
    //cv::ellipse( frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );
    cv::Point pt1(faces[i].x,faces[i].y);
    cv::Point pt2(faces[i].x+faces[i].width,faces[i].y+faces[i].height);
	cv::rectangle(frame,pt1,pt2,cv::Scalar( 0, 255, 0 ),4,8,0);
    cv::Mat faceROI = frame_gray( faces[i] );
    
    
    std::vector<cv::Rect> eyes;


    //-- In each face, detect eyes
    eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

    for( size_t j = 0; j < eyes.size(); j++ )
     {
       cv::Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
       int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
       cv::circle( frame, center, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
     }

	 cv::Mat recoFace;
	 cv::Size faceROI_size = faceROI.size();
	 if(faceROI_size.height*faceROI_size.width>recoFace_size.height*recoFace_size.width){
		cv::cvtColor( faceROI, recoFace, cv::COLOR_GRAY2RGB );
	    recoFace_size = recoFace.size();
		cv::imshow("Ammazza che bono", recoFace);
	}	 

  }
  
  
  if(faces.size()==0){
	  profile_face_cascade.detectMultiScale( frame_gray, profile, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

	  
	  for( size_t i = 0; i < profile.size(); i++ )
	  {
		//cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
		//cv::ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
		cv::Point pt1(profile[i].x,profile[i].y);
		cv::Point pt2(profile[i].x+profile[i].width,profile[i].y+profile[i].height);
		cv::rectangle(frame,pt1,pt2,cv::Scalar( 0, 0, 255 ),4,8,0);

		//Mat faceROI = frame_gray( faces[i] );
		
	  }
  }
  
  

  //-- Show what you got
  cv::imshow( "Te sto a guarda'", frame );
  
  return recoFace_size;

 }
