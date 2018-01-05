#include "Camera.hpp"
#include <iostream>
#include <opencv2/core.hpp>

#include <chrono>
#include <sys/time.h>

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

using namespace std::chrono;


//Constructors
Camera::Camera(int id):id(id)
{}

Camera::Camera(int id, const char* intrinsics_file):id(id)
{
	
	if(intrinsics_file != NULL)
	{
		if(!readParamsFromFile(intrinsics_file) )
		{
			std::cerr << "Unable to read intrinsic parameters.!\n"; // exit ?
		}
	}
	
}



bool Camera::readParamsFromFile(const char* intrinsics_file)
{
	// Read some parameters from the input file
	cv::FileStorage fs;
	fs.open(intrinsics_file, cv::FileStorage::READ);	

	if(!fs.isOpened())
	{
		return false;
	}
	
	// check if FileNode is empty ?
	fs["camera_matrix"] >> this->cameraMatrix ;
	fs["distortion_coefficients"] >> this->distCoeffs;

	fs.release();

	return true;  

}


// Setters 
void Camera::updateIntrinsicParams(cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
	this->cameraMatrix = cameraMatrix;
	this->distCoeffs = distCoeffs;
}





// Helpers


// =================== Camera related ============================



// saves a video on a particular webcam id
// stores video as .avi format, using XVID codec
void Camera::saveVideo(std::string outputFile)
{
	cv::Mat frame;
	cv::VideoCapture vid(this->id);

	//Define the codec and create VideoWriter object
	int fourcc_codec=CV_FOURCC('X','V','I','D');
	cv::VideoWriter out(outputFile,fourcc_codec, 20.0, cv::Size(640,480));

	while(vid.read(frame))
	{ 
	    out.write(frame);

	    cv::imshow("frame",frame);
	
			char ch = cv::waitKey(1);
	    if(ch == 'q' || ch == 27)
	        break;
	}

	// Release resources.
	vid.release(); out.release();
	cv::destroyAllWindows();
}




int Camera::getFPS() const
{
	int fps;
	double seconds;
	cv::VideoCapture vid(this->id);

	int num_frames = 120; // this can be any number

	cv::Mat temp;
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
  
	// Grab a few frames
	for(int i=0; i < num_frames; ++i)
	{
		vid.read(temp);
	}
		
	//End time
	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	seconds =  std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
	fps  = num_frames / seconds;

	vid.release();
	return fps;

}

