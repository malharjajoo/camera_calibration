#include <iostream>

#include <cmath>




#include "StereoCamera.hpp"
#include "globals.hpp"



using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


//========================= Top Level Function ===========================



int main( int argc, const char** argv )
{
	// Files containing intrinsics ( Optional input to cv::stereoCalibrate )
	char* internal_calibFile_left = "/home/malhar/FYP_opencv/saved_camera_params/logitech_microsoft/logitech_camera_params.xml";
	char* internal_calibFile_right = "/home/malhar/FYP_opencv/saved_camera_params/logitech_microsoft/microsoft_lifecam.xml";
	// files containing intrinsics and extrinsics of both cameras
	std::string intrinsics_file = "Intrinsics.xml"; // These will be improvised intrinsics, byproduct of stereoCalibrate
	std::string extrinsics_file = "Extrinsics.xml"; // Main output of stereoCalibrate & stereoRectify
	
	int do_stereoCalibration = 1; // by default no stereo calibration
	int do_rectification = 1;    // by default no stereo rectifcation

	char* ch = getenv("STEREO_RECTIFY");
	if(ch!=NULL)
	{
		do_rectification = atoi(ch);
	}

	ch = getenv("STEREO_CALIB");
 	if(ch != NULL)
	{
		do_stereoCalibration = atoi(ch);
	}
	

	Camera cam_left(1,internal_calibFile_left);
	Camera cam_right(2,internal_calibFile_right);
	StereoCamera ster_cam(cam_left, cam_right, intrinsics_file, extrinsics_file);


	if(do_stereoCalibration)
	{
		bool foundImagePoints =  ster_cam.load_image_points(); 

		// We run calibration only if we are able to find corners in the input image of chessboard
		if(foundImagePoints)
		{
			std::cerr << " Image points have been succesfully detected\n";
			ster_cam.runAndSaveStereoCalibration();
		}
	}
	
	
	if(do_rectification)
	{
		ster_cam.rectifyImages(true);
	}

	// Try either of the two - maybe do triangulation just to see how value of 3D points
	ster_cam.doTriangulate();

	//doDisparity(intrinsics_file,extrinsics_file);

	std::cerr << "Finished." << "\n";

	return 0;

}

