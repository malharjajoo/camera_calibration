#include <iostream>

#include <cmath>

#include "Detector.hpp"
#include "StereoCamera.hpp"
#include "globals.hpp"

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



using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

#define SAVE_VIDEO 1

cv::Mat reduceBrightness(cv::Mat img);

//========================= Helper Functions ===========================

// get average of all triangulation points
cv::Vec3f processTriangulation(cv::Mat triangPoints)
{
	cv::Vec3f mean_coord;
	cv::Scalar mean_value = cv::mean(triangPoints);
	
	mean_coord.val[0] = mean_value.val[0];
	mean_coord.val[1] = mean_value.val[1];
	mean_coord.val[2] = mean_value.val[2];

	//float x = mean_coord.val[0];
	//float y = mean_coord.val[1];
	//float z = mean_coord.val[2];
	//fprintf(stderr,"Mean coordinates is= (%f,%f,%f)\n",x,y,z);
	
	return mean_coord;
}


// sets all pixels outside the rectangle to value =0 (black).
cv::Mat getBoxOnly(cv::Mat img, cv::Rect rect)
{	
	cv::Mat boxOnly = img.clone();
	
	// set all pixels outside rectangle to 0.
	int rows = img.rows;
	int cols = img.cols;
	for(int y = 0 ; y < rows; ++y)
	{
		for(int x = 0 ;x < cols ; ++x)
		{
			if(!rect.contains(cv::Point(x,y)))
			{
				boxOnly.at<Vec3b>(y, x) = boxOnly.at<Vec3b>(y, x) * 0 ;
			}
		}
	}

	return boxOnly;
	
}



// Finds drone coordinates by triangulating all points 
// within a bounding rectangle.
// Change this to return a a single/array of (x,y,z) coordinates
void findDroneCoordinates(StereoCamera ster_cam)
{
	/* 
	cv::Mat dup_frame_left, dup_frame_right;
	std::string outputFile_left = "drone_left.avi";
	std::string outputFile_right = "drone_right.avi";

	int fourcc_codec=CV_FOURCC('X','V','I','D');
	cv::VideoWriter out1(outputFile_left,fourcc_codec, 20.0, cv::Size(640,480));
	cv::VideoWriter out2(outputFile_right,fourcc_codec, 20.0, cv::Size(640,480));
	*/

	std::stringstream ss; 
	//========================
	int id_left, id_right;
	ster_cam.getCameraIDs(&id_left, &id_right);

	cv::VideoCapture vid1(id_left);
	cv::VideoCapture vid2(id_right);


	#if SAVE_VIDEO == 1

		cv::VideoWriter vidWriter;
		std::string outputVideoName = "output.avi";
		int fourcc = static_cast<int>(vid1.get(CV_CAP_PROP_FOURCC)); 
		cv::Size S = Size((int) vid1.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
	                  (int) vid1.get(CV_CAP_PROP_FRAME_HEIGHT));

		double fps = 10.0;
		vidWriter.open(outputVideoName,CV_FOURCC('M','J','P','G'), fps , S, true);

	#endif

	cv::Mat img1,img2;
	cv::Mat boxOnly1, boxOnly2;

	std::string left_camera_window = "Triangle - Left camera view";
	std::string right_camera_window = "Triangle - Right camera view";
	cv::namedWindow(left_camera_window);
	cv::namedWindow(right_camera_window);
	cv::moveWindow(left_camera_window, 20,20);
	cv::moveWindow(right_camera_window, 700,20);

	//1) Do background initialization for both cameras.
	DroneDetector* detect_left  = new DroneDetector() ; 
	DroneDetector* detect_right = new DroneDetector(); 

	detect_left->initializeBackground( detect_left->getBackgroundImage(vid1) );
	detect_right->initializeBackground( reduceBrightness(detect_right->getBackgroundImage(vid2)) );

	
	/* 
	createHSVTrackBar("HSV_trackbar");
	Detector* detect_left  = new ThreatDetector();
	Detector* detect_right = new ThreatDetector();
	*/

	cv::Rect rect_left;  // bounding box for object in left image.
	cv::Rect rect_right; // bounding box for object in right image.

  	detect_left->displayInfo();detect_right->displayInfo(); cv::destroyAllWindows(); // to display background images
	
	char key;
	int wait_delay = 5;

	while(vid1.read(img1) && vid2.read(img2) )
	{
		// doing this only because of issues with lighting in second webcam.
		img2 = reduceBrightness(img2);
		//dup_frame_left  = img1.clone();dup_frame_right = img2.clone();

		//2) Pass it both left and right frames, one by one
		rect_left  = detect_left->detect(img1);
		rect_right = detect_right->detect(img2);

		// only allow if left and right rectangles are reasonably similar
		// in size ??
		if( abs(rect_left.area() - rect_right.area()) < 500 )
		{ 

			// 3) Extract ROI or blacken all other image region so that no features can 
			// be found there and then triangulate in given region.
			boxOnly1 = getBoxOnly(img1,rect_left);
			boxOnly2 = getBoxOnly(img2,rect_right);

			// process the output of triangulation - average ( or median ?)
			cv::Mat triangPoints = ster_cam.doTriangulate_SingleFrame(boxOnly1,boxOnly2);
			
			if(!triangPoints.empty())
			{
				//std::cerr << "Got triangulation output" << "\n";
				cv::Vec3f summary_coord = processTriangulation(triangPoints);
				
				// display the 3D coordinates in the middle of the rectangle
				std::stringstream ss;
				ss << "( " << summary_coord.val[0] <<" , " << summary_coord.val[1] << " , " << summary_coord.val[2] <<" )" ;
				cv::Point2f rect_centre = Point2f( float(rect_left.tl().x + rect_left.br().x) / 2.0 , float(rect_left.tl().y + rect_left.br().y) / 2.0 );

				cv::circle(img1,rect_centre, 3, GREEN, -1, CV_AA);
				cv::putText(img1,ss.str(),rect_centre, cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, RED);
				cv::putText(img1,"[DRONE]",Point2f(rect_centre.x-10,rect_left.y-10), cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE+0.1, RED);
			}
			
			//cv::imshow("boxOnly1", boxOnly1);
			//cv::imshow("boxOnly2", boxOnly2);
		}

		else
		{
			//std::cerr << "Picture sizes are not comparable!\n";
		}
		cv::imshow(left_camera_window, img1);
		cv::imshow(right_camera_window, img2);
			
		#if SAVE_VIDEO == 1

			vidWriter << img1;

		#endif


		key = (char)waitKey(wait_delay);
	  
		if(key == 'q' || key == esc_ascii)
		{
			break;
		}

		
	}

	cv::destroyAllWindows();
	vid1.release(); vid2.release();

}


//========================= Top Level Function ===========================


int main( int argc, const char** argv )
{
	// Files containing intrinsics ( Optional input to cv::stereoCalibrate )
	//char* internal_calibFile_left = "/home/malhar/FYP_opencv/saved_camera_params/logitech_microsoft/logitech_camera_params.xml";
	char* internal_calibFile_left  = "/home/malhar/FYP_opencv/saved_camera_params/logitech_microsoft/out_camera_data_logitech.xml";
	char* internal_calibFile_right = "/home/malhar/FYP_opencv/saved_camera_params/logitech_microsoft/microsoft_lifecam.xml";
	
	// files containing intrinsics and extrinsics of both cameras
	std::string intrinsics_file = "Intrinsics.xml"; // improvised intrinsics, byproduct of stereoCalibrate
	std::string extrinsics_file = "Extrinsics.xml"; // Main output of stereoCalibrate & stereoRectify
	
	int do_stereoCalibration = 0; // by default no stereo Calibration
	int do_rectification = 0;     // by default no stereo Rectifcation

	char* ch = getenv("STEREO_CALIB");
 	if(ch != NULL)
	{
		do_stereoCalibration = atoi(ch);
	}

	ch = getenv("STEREO_RECTIFY");
	if(ch!=NULL)
	{
		do_rectification = atoi(ch);
	}
	

	// Initialize camera system.
	Camera cam_left(1,internal_calibFile_left);
	Camera cam_right(2,internal_calibFile_right);
	StereoCamera ster_cam(cam_left, cam_right, intrinsics_file, extrinsics_file);


	if(do_stereoCalibration)
	{
		bool foundImagePoints = ster_cam.load_image_points(); 

		// calibration run only if we are able to find features/corners in the
		// input image pairs (OpenCV requires a chessboard image for calibration)
		if(foundImagePoints)
		{
			std::cerr << " Image points have been succesfully detected\n";
			ster_cam.runAndSaveStereoCalibration();
		}
	}
	
	
	if(do_rectification)
	{
		bool display = true;
		ster_cam.rectifyImages(display);
	}

	// Try either of the two - triangulation or disparity map.
	// If both cameras use same algorithm, this can be made in OOP way. 
	// make an initializer for both threat and drone.
	findDroneCoordinates(ster_cam);
	
	//ster_cam.doTriangulate();

	//ster_cam.doDisparity(intrinsics_file.c_str(),extrinsics_file.c_str());

	std::cerr << "Finished." << "\n";

	return 0;

}

