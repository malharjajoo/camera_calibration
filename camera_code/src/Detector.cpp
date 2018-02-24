
		
#include <iostream>
#include <fstream>

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/calib3d.hpp>

#include <opencv2/video/background_segm.hpp>
#include "opencv2/bgsegm.hpp"


#include "globals.hpp"
#include "Detector.hpp"


using namespace cv;
using namespace std;



// OpenCV provides convenient methods for videos but not for webcam.
int getFrameInformation_Video(cv::VideoCapture vid)
{
	return vid.get(cv::CAP_PROP_FPS);
}



void displayFrameInformation(cv::VideoCapture vid,cv::Mat frame,int fps)
{
	int frame_number = vid.get(cv::CAP_PROP_POS_FRAMES);
	std::stringstream ss; 
	ss << "Frame_no:" << frame_number;
	cv::putText(frame,ss.str(),cv::Point(20,15),cv::FONT_HERSHEY_SIMPLEX,FONT_SIZE,RED);
	ss.str(""); ss << "fps:" << fps;
	cv::putText(frame,ss.str(),cv::Point(20,30),cv::FONT_HERSHEY_SIMPLEX,FONT_SIZE,RED);
}


// ======= Morphology Trackbar related ======

void CreateBGSUBTrackBar()
{
	int value =1;
	cv::namedWindow("bg_sub");
	cv::createTrackbar("shape","bg_sub",&value,3);
	cv::createTrackbar("kernel_size","bg_sub",&value,10);
	cv::createTrackbar("iterations","bg_sub",&value,10);
}

void getParams(cv::Mat kernel, int& iterations)
{
	int shape = -10;
	int val =  cv::getTrackbarPos("shape","bg_sub");
	
	if(val==1)
		shape = cv::MORPH_RECT;
	else if(val==2)
		shape = cv::MORPH_ELLIPSE;
	else
		shape = cv::MORPH_CROSS;

	int sz = cv::getTrackbarPos("kernel_size","bg_sub");

	// get the kernel based on shape and size
	kernel = cv::getStructuringElement(shape,cv::Size(sz,sz));
	iterations = cv::getTrackbarPos("iterations","bg_sub");

}




//=============== Common functions ===========

// Draw bounding rectangle in a coloured image:
// Note: input_image must be a binary 8 bit image (probably as a result of thresholding).	
cv::Rect getRectangle(cv::Mat input_image, cv::Mat output_image, bool display=true)
{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(input_image,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
	int len = contours.size();

	cv::Rect rect;

	// If image is completely dark (no object was detected), and no contours are found.
	if(len > 0)
	{
		// filter contours and choose contour with max points since we want to avoid
		// small inner rectangles for objects with holes
		int contour_length_max = 0;
		int countour_idx = 1;

		for(int i=0; i < len ; ++i)
		{
			if (contours[i].size() > contour_length_max)
			{
				contour_length_max = contours[i].size();
			    countour_idx = i;
			}
				
		}
	
		//draw rectangle for only longest contour
		rect = cv::boundingRect(contours[countour_idx]);

		if(display)
		{
			cv::rectangle(output_image,rect.tl(), rect.br(),BLUE,3);
		}
		
	}

	return rect;
}




//=============== Drone Related Functions =============


DroneDetector::DroneDetector()
{
}

void DroneDetector::initializeBackground(cv::Mat bg_img)
{
	this->background_img = bg_img;
}

void DroneDetector::displayInfo()
{
	cv::imshow("background", this->background_img);
	char key = (char)waitKey(0);
	cv::destroyAllWindows();
}


cv::Mat DroneDetector::getBackgroundImage(cv::VideoCapture vid)
{
	cv::Mat temp,background_img;

	// Dont use 1st frame as background
	for(int i=0;i<3;++i)
	{vid.read(temp);}
	vid.read(background_img);

	return background_img;
}


// Does bgSub for single frame
// draws box in input frame. NOte: all cv::Mat are passed by refernce in openCV,
cv::Rect DroneDetector::detect(cv::Mat frame)
{
	int threshold = 40;

	// Get the difference image, convert to grayscale and threshold it.
	cv::Mat diff;
	cv::absdiff(frame, this->background_img, diff);

	cv::Mat gray_diff;
	cv::cvtColor(diff,gray_diff,cv::COLOR_BGR2GRAY);
	
	cv::Mat thresh_img;
	cv::threshold(gray_diff,thresh_img,threshold,255,cv::THRESH_BINARY);

	// Display frame rate,etc
	//displayFrameInformation(vid,frame,fps);

	return getRectangle(thresh_img,frame);
}

/*
// This function explores different Background Subtraction methods.
void detect(cv::VideoCapture vid)
{
	cv::Mat background_img,frame,diff;
	//Ptr<cv::BackgroundSubtractor> fgbg;
	
	CreateBGSUBTrackBar();

	// choose 1 of the 3 methods.
	//fgbg = bgsegm::createBackgroundSubtractorMOG();
	//fgbg = cv::createBackgroundSubtractorMOG2();
	//fgbg = cv::bgsegm::createBackgroundSubtractorGMG();
	
	background_img = this->background_img;

	int fps=getFrameInformation_Video(vid);
	
	int threshold = 40;

	cv::Mat fgmask;
	cv::Mat kernel;
	int iterations;
	int operation = cv::MORPH_OPEN; //cv::MORPH_ERODE
	cv::Rect rect;

	while(vid.read(frame)){
	
		// Get the difference image, convert to grayscale and threshold it.
		cv::absdiff(frame, background_img, diff);
		//cv::imshow("diff",diff);

		cv::Mat gray_diff;
		cv::cvtColor(diff,gray_diff,cv::COLOR_BGR2GRAY);
		//cv::imshow("gray_diff",gray_diff);
		
		
		cv::Mat thresh_img;
		cv::threshold(gray_diff,thresh_img,threshold,255,cv::THRESH_BINARY);
		//cv::imshow("difference",thresh_img);

		//getParams(kernel,iterations);cv::morphologyEx(thresh_img,thresh_img,operation,kernel,Point(-1,-1),iterations=iterations);cv::imshow("morphed",thresh_img);
	

		// Display frame rate,etc
		displayFrameInformation(vid,frame,fps);

		rect = drawRectangle(thresh_img,frame);
		cv::imshow("original",frame);

		//fgbg->apply(frame,fgmask);cv::imshow("MOG",fgmask);
		
		
		char k = cv::waitKey(50) & 0xff;
		if (k == 27){break;}
	}

	vid.release();
	cv::destroyAllWindows();
}

*/

//===================== Threat Related Functions =============

 int h_min = 0 ; 
 int h_max = 179; 
 int s_min = 0 ;
 int s_max = 255;
 int v_min = 0; 
 int v_max = 255;  


// since openCV only allows us to create 1 trackbar at a time,
// easier to group all trackbars into 1 function.s
void createHSVTrackBar(const char* trackBarWindowName) 
{
	cv::namedWindow(trackBarWindowName);

	createTrackbar("H_min",trackBarWindowName,&h_min,h_max,0);
	createTrackbar("H_max",trackBarWindowName,&h_max,h_max,0) ;
	createTrackbar("S_min",trackBarWindowName,&s_min,s_max,0) ;
	createTrackbar("S_max",trackBarWindowName,&s_max,s_max,0) ;
	createTrackbar("V_min",trackBarWindowName,&v_min,v_max,0) ;
	createTrackbar("V_max",trackBarWindowName,&v_max,v_max,0) ;

}


// Can display various thresholds used.
void ThreatDetector::displayInfo()
{
	std::cerr << "Using HSV for object detection\n";
}



// Uses Colour Based Segmentation
cv::Rect ThreatDetector::detect(cv::Mat frame)
{
	// Change color space
	cv::Mat hsv_frame;
	cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

	// threshold image based on range
	cv::Scalar min_hsv = Scalar(h_min,s_min,v_min);
	cv::Scalar max_hsv = Scalar(h_max,s_max,v_max);
	cv::inRange(hsv_frame,min_hsv,max_hsv,hsv_frame);

	// find bounding rectangle and display in image (2nd param)
	cv::Rect rect = getRectangle(hsv_frame,frame);
	return rect;

}
