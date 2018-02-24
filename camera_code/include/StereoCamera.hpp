#ifndef STEREOCAMERA
#define STEREOCAMERA

#include "Camera.hpp"

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


/*
class ExtrinsicParams
{
	public:
		// extrinsics/pose of stereo camera setup.
		cv::Mat R,T,F;	
		double rms;

		// output of rectification process
		cv::Mat R1,P1,R2,P2,Q; 

		void update_pose_and_error();
		
};
*/

class StereoCamera
{
	private:

		Camera left_camera,right_camera;

		// extrinsics/pose of stereo camera setup.
		cv::Mat R,T,F;
		double rms;
		cv::Mat R1,P1,R2,P2,Q; // output of rectification process
		
		cv::Size imageSize; // should this be part of stereo camera setup ?
		std::string intrinsics_file, extrinsics_file;

		std::vector<cv::Mat> goodGrayImageList; // list of "pair" of images where features/corners were detcted
		std::vector< std::vector< cv::Point2f > > left_img_points, right_img_points; //stores all corners' coordinates.
		

	public:

		// Constructor 
		StereoCamera(Camera left_camera, Camera right_camera, std::string intrinsics_file,std::string extrinsics_file);
		bool readParamsFromFile(const char* extrinsics_file);
	
		// Getters and Setters
		void getCameraIDs(int* id_left, int* id_right) const;

		void updateIntrinsicParams(cv::Mat cameraMatrix1,cv::Mat distCoeffs1,cv::Mat cameraMatrix2, cv::Mat distCoeffs2,bool saveToFile=false);
		void saveIntrinsicParametersToFile(cv::Mat cameraMatrix1, cv::Mat cameraMatrix2, cv::Mat distCoeffs1, cv::Mat distCoeffs2);


		void updateExtrinsicParams(cv::Mat R,cv::Mat T, cv::Mat F,double rms,bool saveToFile=false);
		void saveExtrinsicParametersToFile(cv::Mat R, cv::Mat T,cv::Mat F,double rms);
		void appendSaveRectificationParameters();

		void updateRectificationParams(cv::Mat R1, cv::Mat P1,cv::Mat R2,cv::Mat P2,cv::Mat Q,
		bool saveToFile=false);
	
		// Imp functions
		bool load_image_points(int num_imgs=50);
		int runAndSaveStereoCalibration(); 
		int rectifyImages(bool display=false);
		bool findCorrespondingFeaturesBothImages(cv::Mat img1,cv::Mat img2,std::vector< cv::Point2f >& observedCorners1, std::vector< cv::Point2f >& observedCorners2,bool corner_refine=true, bool display=false);
		int doTriangulate();
		cv::Mat doTriangulate_SingleFrame(cv::Mat img1, cv::Mat img2, bool display=false);
		void doDisparity(const char* intrinsics_file, const char* extrinsics_file);
};


#endif
