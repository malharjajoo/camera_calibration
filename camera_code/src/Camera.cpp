#include "Camera.hpp"
#include <iostream>
#include <opencv2/core.hpp>

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

