#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <stdlib.h>
using namespace std;
using namespace cv;

// trackbar variables 
// It is imp to remember that openCV uses a particular range 
// for h,s,v values.
// h: { 0-180}, s:{0-255} , v:{0-255}
int h_min = 0 ; 
int h_max = 180; 
int s_min = 0 ;
int s_max = 255;
int v_min = 0; 
int v_max = 255;  

int alpha = 10;
int alpha_max = 30;
int beta = 0;
int beta_max = 255;

cv::Mat frame1,frame2;
cv::Mat hsv_frame1,hsv_frame2;
cv::Mat lab_frame1,lab_frame2;
cv::Mat ycrcb_frame1,ycrcb_frame2;
char* trackBarWindowName = "trackBar_window" ;


//=======================================================================



// this is lot of redundant code .... but using it to support colour detection 
// for both windows .... need to remove, only issue is not sure how to pass 
// more parameters to openCV mouseHandler ?!
void setLabel2(cv::Mat im, const cv::Point& orig, Vec3b bgrPixel, Vec3b hsvPixel, Vec3b ycbPixel, Vec3b labPixel)
{
		std::stringstream ss1,ss2,ss3,ss4;
		ss1 << "BGR: " << "[" << (int)bgrPixel.val[0] << "," << (int)bgrPixel.val[1] << "," 
					<< (int)bgrPixel.val[2] << "]" ;
		fprintf(stderr,"BGR:[%d, %d, %d]\n",
    (int)bgrPixel.val[0],(int)bgrPixel.val[1],(int)bgrPixel.val[2]);

		 ss2 << "HSV: " << "[" << (int)hsvPixel.val[0] << "," << (int)hsvPixel.val[1] << "," 
			<< (int)hsvPixel.val[2] << "]" ;
		 fprintf(stderr,"HSV:[%d, %d, %d]\n",
    (int)hsvPixel.val[0],(int)hsvPixel.val[1],(int)hsvPixel.val[2]);

		 ss3 << "LAB: " << "[" << (int)labPixel.val[0] << "," << (int)labPixel.val[1] << "," 
			<< (int)labPixel.val[2] << "]" ;
		 fprintf(stderr,"LAB:[%d, %d, %d]\n",
    (int)labPixel.val[0],(int)labPixel.val[1],(int)labPixel.val[2]);

		ss4 << "YCrCb: " << "[" << (int)ycbPixel.val[0] << "," << (int)ycbPixel.val[1] << "," 
			<< (int)ycbPixel.val[2] << "]" ;
		 fprintf(stderr,"YCrCb:[%d, %d, %d]\n\n",
    (int)ycbPixel.val[0],(int)ycbPixel.val[1],(int)ycbPixel.val[2]);

		

    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.5;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(ss1.str()+ss2.str(), fontface, scale, thickness, &baseline);
    cv::rectangle(im, orig , orig + cv::Point(0.7*text.width, 8*text.height), CV_RGB(0,0,0), CV_FILLED);
		
    cv::putText(im, ss1.str(), orig + cv::Point(0, 1.6*text.height), fontface, scale, CV_RGB(255,255,255), thickness, 8);
		cv::putText(im, ss2.str(), orig + cv::Point(0, 3.2*text.height), fontface, scale, CV_RGB(255,255,255), thickness, 8);
		cv::putText(im, ss3.str(), orig + cv::Point(0, 4.8*text.height), fontface, scale, CV_RGB(255,255,255), thickness, 8);
		cv::putText(im, ss4.str(), orig + cv::Point(0, 6.4*text.height), fontface, scale, CV_RGB(255,255,255), thickness, 8);
		

		cv::imshow("Original image right", im);

}

// this is lot of redundant code .... but using it to support colour detection 
// for both windows .... need to remove, only issue is not sure how to pass 
// more parameters to openCV mouseHandler ?!
void mouseHandler2(int event,int x,int y, int flags,void* img)
{
		
		Mat* rgb_img = (Mat*)img;
		
		if(event == EVENT_LBUTTONDBLCLK)
		{
				//Convert to other color spaces 
				Vec3b bgrPixel = (*rgb_img).at<Vec3b>(y,x);
				Mat3b bgr (bgrPixel);
			
				Mat3b hsv,ycb,lab;
				cvtColor(bgr, ycb, COLOR_BGR2YCrCb);
				cvtColor(bgr, hsv, COLOR_BGR2HSV);
				cvtColor(bgr, lab, COLOR_BGR2Lab);
							
				Vec3b hsvPixel(hsv.at<Vec3b>(0,0));
				Vec3b ycbPixel(ycb.at<Vec3b>(0,0));
				Vec3b labPixel(lab.at<Vec3b>(0,0));
	
				// Print them all 
				fprintf(stderr,"coordinates:(%d, %d)\n",x,y ); 

				cv::circle(*rgb_img,Point(x,y),5,Scalar(255,0,255),-1);
				setLabel2(*rgb_img, Point(20,20),bgrPixel,hsvPixel,ycbPixel, labPixel);
				
		}
			
		
}



void setLabel(cv::Mat im, const cv::Point& orig, Vec3b bgrPixel, Vec3b hsvPixel, Vec3b ycbPixel, Vec3b labPixel)
{
		std::stringstream ss1,ss2,ss3,ss4;
		ss1 << "BGR: " << "[" << (int)bgrPixel.val[0] << "," << (int)bgrPixel.val[1] << "," 
					<< (int)bgrPixel.val[2] << "]" ;
		fprintf(stderr,"BGR:[%d, %d, %d]\n",
    (int)bgrPixel.val[0],(int)bgrPixel.val[1],(int)bgrPixel.val[2]);

		 ss2 << "HSV: " << "[" << (int)hsvPixel.val[0] << "," << (int)hsvPixel.val[1] << "," 
			<< (int)hsvPixel.val[2] << "]" ;
		 fprintf(stderr,"HSV:[%d, %d, %d]\n",
    (int)hsvPixel.val[0],(int)hsvPixel.val[1],(int)hsvPixel.val[2]);

		 ss3 << "LAB: " << "[" << (int)labPixel.val[0] << "," << (int)labPixel.val[1] << "," 
			<< (int)labPixel.val[2] << "]" ;
		 fprintf(stderr,"LAB:[%d, %d, %d]\n",
    (int)labPixel.val[0],(int)labPixel.val[1],(int)labPixel.val[2]);

		ss4 << "YCrCb: " << "[" << (int)ycbPixel.val[0] << "," << (int)ycbPixel.val[1] << "," 
			<< (int)ycbPixel.val[2] << "]" ;
		 fprintf(stderr,"YCrCb:[%d, %d, %d]\n\n",
    (int)ycbPixel.val[0],(int)ycbPixel.val[1],(int)ycbPixel.val[2]);

		

    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.5;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(ss1.str()+ss2.str(), fontface, scale, thickness, &baseline);
    cv::rectangle(im, orig , orig + cv::Point(0.7*text.width, 8*text.height), CV_RGB(0,0,0), CV_FILLED);
		
    cv::putText(im, ss1.str(), orig + cv::Point(0, 1.6*text.height), fontface, scale, CV_RGB(255,255,255), thickness, 8);
		cv::putText(im, ss2.str(), orig + cv::Point(0, 3.2*text.height), fontface, scale, CV_RGB(255,255,255), thickness, 8);
		cv::putText(im, ss3.str(), orig + cv::Point(0, 4.8*text.height), fontface, scale, CV_RGB(255,255,255), thickness, 8);
		cv::putText(im, ss4.str(), orig + cv::Point(0, 6.4*text.height), fontface, scale, CV_RGB(255,255,255), thickness, 8);
		

		cv::imshow("Original image left", im);

}

//==================================================================


// x and y is coordinate of mouse pointer.
void mouseHandler(int event,int x,int y, int flags,void* img)
{
		
		Mat* rgb_img = (Mat*)img;
		
		if(event == EVENT_LBUTTONDBLCLK)
		{
				//Convert to other color spaces 
				Vec3b bgrPixel = (*rgb_img).at<Vec3b>(y,x);
				Mat3b bgr (bgrPixel);
			
				Mat3b hsv,ycb,lab;
				cvtColor(bgr, ycb, COLOR_BGR2YCrCb);
				cvtColor(bgr, hsv, COLOR_BGR2HSV);
				cvtColor(bgr, lab, COLOR_BGR2Lab);
							
				Vec3b hsvPixel(hsv.at<Vec3b>(0,0));
				Vec3b ycbPixel(ycb.at<Vec3b>(0,0));
				Vec3b labPixel(lab.at<Vec3b>(0,0));
	
				// Print them all 
				fprintf(stderr,"coordinates:(%d, %d)\n",x,y ); 

				cv::circle(*rgb_img,Point(x,y),5,Scalar(255,0,255),-1);
				setLabel(*rgb_img, Point(20,20),bgrPixel,hsvPixel,ycbPixel, labPixel);
				
		}
			
		
}



// since openCV only allows us to create 1 trackbar at a time,
// easier to group all trackbars into 1 function
void createHSVTrackBar(const char* trackBarWindowName) 
{

	createTrackbar("H_min",trackBarWindowName,&h_min,h_max,0);
	createTrackbar("H_max",trackBarWindowName,&h_max,h_max,0) ;
	createTrackbar("S_min",trackBarWindowName,&s_min,s_max,0) ;
	createTrackbar("S_max",trackBarWindowName,&s_max,s_max,0) ;
	createTrackbar("V_min",trackBarWindowName,&v_min,v_max,0) ;
	createTrackbar("V_max",trackBarWindowName,&v_max,v_max,0) ;

	createTrackbar("alpha",trackBarWindowName,&alpha,alpha_max,0) ;
	createTrackbar("beta",trackBarWindowName,&beta,beta_max,0) ;

}

//int WIDTH = 1000; 
int main(int argc,char** argv) 
{
	cv::namedWindow(trackBarWindowName/*,WINDOW_AUTOSIZE*/);
	createHSVTrackBar(trackBarWindowName);

	cv::VideoCapture vid1(1); cv::VideoCapture vid2(2);
	
 	/*
	int width = int(vid1.get(CV_CAP_PROP_FRAME_WIDTH));
	int height = int(vid1.get(CV_CAP_PROP_FRAME_HEIGHT));
	vid1.set(CV_CAP_PROP_FRAME_WIDTH , width);
	vid1.set(CV_CAP_PROP_FRAME_HEIGHT , 470);
	cout << "height and width are = " << "(" << width << "," << height << ")" << "\n";
	*/


	if(!vid1.isOpened() || !vid2.isOpened())
	{
		std::cerr << "Unable to open video !" << "\n";
		return -1;
	}
		
	std::string window_name1 = "Original image left";
	std::string window_name2 = "Original image right";
	namedWindow(window_name1); namedWindow(window_name2);

	while(vid1.read(frame1) && vid2.read(frame2))
	{
		// Show the frame1.
		double alpha_d= (double)alpha/10.0;
		frame2.convertTo(frame2, -1, alpha_d, -1*beta);

		setMouseCallback(window_name1, mouseHandler, &frame1 ); 
		setMouseCallback(window_name2, mouseHandler2, &frame2 ); 
	
		imshow("Original image left", frame1);
		imshow("Original image right",frame2);
		
		//1) convert image to various color spaces
		// BGR -> HSV 
		cv::cvtColor(frame1, hsv_frame1, cv::COLOR_BGR2HSV);
		cv::cvtColor(frame2, hsv_frame2, cv::COLOR_BGR2HSV);
		
		Scalar min_hsv = Scalar(h_min,s_min,v_min);
		Scalar max_hsv = Scalar(h_max,s_max,v_max);

		cv::inRange(hsv_frame1,min_hsv,max_hsv,hsv_frame1);
		cv::inRange(hsv_frame2,min_hsv,max_hsv,hsv_frame2);
		
		
		
		imshow("HSV thresholded Left",hsv_frame1);
		imshow("HSV thresholded Right",hsv_frame2);
	

		char ch = waitKey(3000);
		if(ch == 'q')
		{break ; 		}

	}

}


		/*
		// BGR -> YCrCb
		cv::cvtColor(frame1, ycrcb_frame1, cv::COLOR_BGR2YCrCb);
		cv::cvtColor(frame2, ycrcb_frame2, cv::COLOR_BGR2YCrCb);
		
		// BGR -> LAB
		cv::cvtColor(frame1, lab_frame1, cv::COLOR_BGR2LAB);
		cv::cvtColor(frame2, lab_frame2, cv::COLOR_BGR2LAB);
		*/

