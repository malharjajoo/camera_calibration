#ifndef CAMERA
#define CAMERA
#include <opencv2/core.hpp>

class Camera
{
	public:
	
		cv::Mat cameraMatrix, distCoeffs;

	
		int id ; // device id of the camera.

		// Constructor
		Camera(int id);
		Camera(int id, const char* intrinsics_file);

		bool readParamsFromFile(const char* intrinsics_file);

		// Setter
		void updateIntrinsicParams(cv::Mat cameraMatrix, cv::Mat distCoeffs);

};

#endif
