#include "StereoCamera.hpp"
		
#include <iostream>
#include <fstream>
#include "globals.hpp"
#include "/home/malhar/opencv/apps/interactive-calibration/rotationConverters.hpp"

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

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

const int wait_key_delay = 1500;

//==================== Helper functions ===================


void StereoCamera::getCameraIDs(int* id_left, int* id_right) const
{
	(*id_left) = this->left_camera.id;
	(*id_right) = this->right_camera.id;

}


// 1) Display general info on Matrices
// 2) convert single channel input to multi channel cv::Mat
// 3) convert 3x3 rotation matrix to euler coordinates.
// 4) Calulate known board positions for camera calibration


// Computes P = K * [R|t]
// K : cameraMatrix , 3x3
// R : rotationMatrix , 3x3
// t : translation matrix , 3x1
cv::Mat computeProjectionMatrix(cv::Mat K,cv::Mat R,cv::Mat t)
{
	cv::Mat RTMat(3, 4, CV_64F);
	cv::hconcat(R,t,RTMat); //horizontal concatenation

	return K*RTMat;
}


void printMatrixInfo(cv::Mat M)
{
	cout << "rows = " << M.rows << "\n";
	cout << "cols = " << M.cols << "\n";
	cout << "channels = " << M.channels() << "\n";
}


// cv::Mat.size() gives [ width x height ]
// but we want to display row x col
void findSize(cv::Mat mat)
{
	std::cout << "Size of matrix (row x col )= [" << mat.size().height << " x " << mat.size().width <<"]"  << "\n";
}


// If input matrix dimension is ( number of channels x entries )
// Extract each row and then group each entry into a single entry with 
// multiple channels(= entries in std::vector<cv::Mat> channels vector below) 

// imp thing is usage of cv::merge()
cv::Mat convertToMultiChannel(cv::Mat m)
{
	cv::Mat resMultiChannel;  //Output of function
	
	std::vector<cv::Mat> channels;	

	for(unsigned i = 0 ;i < m.size().height ; i++)
	{
		channels.push_back(m.row(i));
	}
	cv::merge(channels, resMultiChannel);
	return resMultiChannel;
}


// convertToMultiChannel() converted input matrix to multi channel matrix
// This function converts vector to 2-channel matrix
cv::Mat convertPoint2fTo2ChannelMatrix(std::vector<Point2f> vec)
{
	cv::Mat xes(vec.size(),1,CV_64FC1);
	cv::Mat yes(vec.size(),1,CV_64FC1);

	for(unsigned i=0; i < vec.size(); ++i)
	{
		xes.at<double>(i,0) = vec[i].x;
		yes.at<double>(i,0) = vec[i].y;
	}
	
	std::vector<cv::Mat> channels;
	channels.push_back(xes);
	channels.push_back(yes);
		
	cv::Mat res2;
	cv::merge(channels,res2);

	return res2;
}




// Converts rotation to Euler angles.
// Please note Euler angles != Rodrigues 
// Input - 3x3 Rotation matrix
// Output - 3x1 Euler Angles ( in degrees )  [ pitch , yaw, roll ]'
void myRot2Euler(const cv::Mat& src, cv::Mat& dst)
{
    if((src.rows == 3) && (src.cols == 3))
    {
        //convert rotaion matrix to 3 angles (pitch, yaw, roll)
        dst = cv::Mat(3, 1, CV_64F);
        double pitch, yaw, roll;

        if(src.at<double>(0,2) < -0.998)
        {
            pitch = -atan2(src.at<double>(1,0), src.at<double>(1,1));
            yaw = -CALIB_PI_2;
            roll = 0.;
        }
        else if(src.at<double>(0,2) > 0.998)
        {
            pitch = atan2(src.at<double>(1,0), src.at<double>(1,1));
            yaw = CALIB_PI_2;
            roll = 0.;
        }
        else
        {
            pitch = atan2(-src.at<double>(1,2), src.at<double>(2,2));
            yaw = asin(src.at<double>(0,2));
            roll = atan2(-src.at<double>(0,1), src.at<double>(0,0));
        }

        // convert to degree
        pitch *= 180./CALIB_PI;
        yaw *= 180./CALIB_PI;
        roll *= 180./CALIB_PI;
       

        dst.at<double>(0,0) = pitch;
        dst.at<double>(1,0) = yaw;
        dst.at<double>(2,0) = roll;
    }
}


// this calculates 3D board positions - hardcoded stuff
// Note this is only calculating for one image.
// It is then replicated for all images/views during calibration.
static void calcKnownBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.clear();

	for( int i = 0; i < boardSize.height; ++i )
	{
		for( int j = 0; j < boardSize.width; ++j )
		{
			corners.push_back(Point3f(j*squareSize, i*squareSize, 0.0f));
		}
	}	   

}



bool fexists(const char *filename)
{
  ifstream ifile(filename);
	if(!ifile)
		return false;
	else
		return true;
  
}





// Usage of this function would only be meaningful for one configuration
// of camera setup.

// NEED TO CHANGE THIS TO EITHER READ FROM DATA MEMBERS?!
// file below refers to extrinsics file.
/*
	If file exists
	Returns true if -
	a) file can be opened but doesn't contain a reprojection error field.
	b) file contains field and current result is better than best result so far.

	Returns false if - 
	a) file can't be opened. 
	b) file contains field and current result is NOT better than best result so far.

	If file doesnt exist, then simply return true so that it can be overwritten.
*/
bool betterThanPreviousReprojectionError(const char* extrinsics_file, double currentReprojectionError)
{

	bool betterThanPrevious = false;

	if(fexists(extrinsics_file))
	{
			FileStorage fs;
			fs.open(extrinsics_file,FileStorage::READ);

			if(fs.isOpened())
			{
				FileNode n = fs["reprojError"] ;

				if(n.type()!=FileNode::NONE)
				{
					double best_reprojError;
					fs["reprojError"] >> best_reprojError;
	
				
					if(currentReprojectionError <  best_reprojError)
					{
						betterThanPrevious = true;
					}
				
				}
				// NOTE - if input file doesnt contain field, it may be overwritten by caller of this function
				else
				{
					betterThanPrevious = true;
					std::cerr << "Empty Node! (file might be overwritten)\n";
				}
	
				fs.release();
			}

			else
			{
				std::cerr << "File can't be opened!\n";
			}

	}
	// if file doesnt exist, we want it to be overwritten (rather new is created)
	else
	{
		betterThanPrevious = true;
	}
	
	return betterThanPrevious;
}


// save camera matrix and distortion coeffs.
// this could have been implemented for each camera but it's just more convenient this way.
void StereoCamera::saveIntrinsicParametersToFile(cv::Mat cameraMatrix1, cv::Mat cameraMatrix2, cv::Mat distCoeffs1, cv::Mat distCoeffs2)
{

  cv::FileStorage fs(this->intrinsics_file.c_str(), FileStorage::WRITE);
	
	if( fs.isOpened() )
  {
      fs << "cameraMatrix1" << cameraMatrix1 << "distCoeffs1" << distCoeffs1 <<
          "cameraMatrix2" << cameraMatrix2 << "distCoeffs2" << distCoeffs2;
      fs.release();
  }
  else
	{
      std::cerr << "Error: can not save the intrinsic parameters ( File cannot be opened )\n";
	}

}



// save relative camera pose...
// overwrites if file exists.
void StereoCamera::saveExtrinsicParametersToFile(cv::Mat R, cv::Mat T,cv::Mat F,double rms)
{
	cv::Mat rotVector_R ;
	myRot2Euler(R,rotVector_R);  // convert to more comprehensible form	
	
	cv::FileStorage fs(this->extrinsics_file.c_str(), FileStorage::WRITE);

  if( fs.isOpened() )
  {
      fs << "reprojError" << rms << "R" << R << "Euler_R" << rotVector_R << "T" << T << "F" << F  ;
      fs.release();
  }
  else
	{
      std::cerr << "Error: can not save the extrinsic parameters after stereo Calibration\n";
	}

}



// Overloaded : save( by "appending" to extrinsics) rectification transformation matrices...
// What if Results already exist ?
// Since openCV doesnt allow duplicate tags in XML file (and doesn't overwrite existing keys),
// and we wish to append results, we need to copy required existing file content and write it back.
void StereoCamera::appendSaveRectificationParameters()
{
	const char* extrinsics_file = this->extrinsics_file.c_str();

	cv::Mat R,rotVector_R,T,F;
	double reprojError;
	
	// read results ( only to copy them back ...)
	FileStorage fs1(extrinsics_file, FileStorage::READ);
	if(fs1.isOpened())
	{
		fs1["R"] >> R ;
		fs1["Euler_R"] >> rotVector_R ;
		fs1["T"] >> T ;
		fs1["F"] >> F ;
		fs1["reprojError"] >> reprojError ;
	}
	fs1.release();

	cv::Mat R1= this->R1.clone();
	cv::Mat R2= this->R2.clone();
	cv::Mat P1= this->P1.clone();
	cv::Mat P2= this->P2.clone();
	cv::Mat Q= this->Q.clone();

	// convert to more comprehensible form	
	cv::Mat rotVector_R1 ;
	cv::Mat rotVector_R2 ;	
	myRot2Euler(R1,rotVector_R1); 
	myRot2Euler(R2,rotVector_R2); 
	
	FileStorage fs(extrinsics_file, FileStorage::WRITE) ; // FileStorage::APPEND
	
    if( fs.isOpened() )
    {
        fs << "reprojError" << reprojError << "R" << R << "Euler_R" << rotVector_R << "T" << T  << "F" << F <<  "R1" << R1 <<  "Euler_R1" << rotVector_R1 << "R2" << R2 << "Euler_R2" << rotVector_R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
	{
        std::cerr << "Error: Can't save the extrinsic parameters after Stereo Rectification!\n";
	}

}





//=================================================================================


bool StereoCamera::readParamsFromFile(const char* extrinsics_file)
{
	// Read some parameters from the input file
	cv::FileStorage fs;
	fs.open(extrinsics_file, cv::FileStorage::READ);	

	if(!fs.isOpened())
	{
		return false;
	}
	
	// check if FileNode is empty ?
	fs["reprojError"] >> this->rms;
	fs["R"] >> this->R ;
	fs["T"] >> this->T;
	fs["F"] >> this->F;

	fs["R1"] >> this->R1 ;
	fs["P1"] >> this->P1;
	fs["R2"] >> this->R2 ;
	fs["P2"] >> this->P2;
	fs["Q"]  >> this->Q;

	fs.release();

	return true;  

}



// constructor 
StereoCamera::StereoCamera(Camera left_camera, Camera right_camera,
													std::string intrinsics_file,std::string extrinsics_file )
:left_camera(left_camera), right_camera(right_camera), intrinsics_file(intrinsics_file),
extrinsics_file(extrinsics_file)
{
		readParamsFromFile(extrinsics_file.c_str());
}



// Setters 

void StereoCamera::updateIntrinsicParams(cv::Mat cameraMatrix1,cv::Mat distCoeffs1,cv::Mat cameraMatrix2, cv::Mat distCoeffs2,
bool saveToFile/*=false*/)
{
		this->left_camera.updateIntrinsicParams(cameraMatrix1,distCoeffs1);
		this->right_camera.updateIntrinsicParams(cameraMatrix2,distCoeffs2);	

		if(saveToFile)
		{
			this->saveIntrinsicParametersToFile(cameraMatrix1,cameraMatrix2,distCoeffs1,distCoeffs2);
		}	
}


void StereoCamera::updateExtrinsicParams(cv::Mat R,cv::Mat T, cv::Mat F,double rms,
bool saveToFile/*=false*/)
{
		this->R = R.clone();
		this->T = T.clone();
		this->F = F.clone();
		this->rms = rms;	

		if(saveToFile)
		{
			this->saveExtrinsicParametersToFile(R,T,F,rms);
		}	
}


void StereoCamera::updateRectificationParams(cv::Mat R1, cv::Mat P1,cv::Mat R2,cv::Mat P2,cv::Mat Q,
bool appendToFile/*=false*/)
{
		// save result --> should I do this ??? had a condition on updating data members earlier.
		this->R1 = R1.clone();
		this->P1 = P1.clone();
		this->R2 = R2.clone();
		this->P2 = P2.clone();
		this->Q  = Q.clone();

	
		if(appendToFile)
		{
			// Appends results to extrinsics file	
			this->appendSaveRectificationParameters(); 
		}	
}







//========================== Loading images and detecting features(corners) ===============================
cv::Mat reduceBrightness(cv::Mat img)
{
	cv::Mat img2;
	img.convertTo(img2, -1, 1, -80);
	return img2;
}
// In order to use this function, images from both cameras must be present.
// It loads images taken from stereo cameras.
// It then detects corners in image pairs and stores the coordinates.
// Also note that the output is stored in a global vector (for convenience)
bool StereoCamera::load_image_points(int num_imgs/*=50*/) 
{
	std::string left_camera_window = "left camera view";
	std::string right_camera_window = "Right camera view";
	cv::namedWindow(left_camera_window);
	cv::namedWindow(right_camera_window);
	cv::moveWindow(left_camera_window, 20,20);
	cv::moveWindow(right_camera_window, 700,20);
  	// These are temp buffers (not entirely sure why I've used these)
	std::vector<vector<cv::Point2f> > imagePoints1,imagePoints2;

	cv::VideoCapture vid1( this->left_camera.id );
	cv::VideoCapture vid2( this->right_camera.id );
	
	if(!vid1.isOpened() || !vid2.isOpened() )
	{
		return false;
	}

	std::cerr << "Reading input images from video stream ....\n";

	// loop over each pair of left and right images.
	for(int i=0;i < num_imgs; )
	{
	
		cv::Mat img1,img2;
		std::vector< cv::Point2f > corners1, corners2;

		if(!vid1.read(img1) || !vid2.read(img2) )
		{
			return false;
		}
		
		img2 = reduceBrightness(img2);
		bool found1 = false, found2 = false;

		// stores corners' coordinates in corners1/corners2 (passed by reference)
		found1 = cv::findChessboardCorners(img1, chessBoardDimension, corners1,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		found2 = cv::findChessboardCorners(img2, chessBoardDimension, corners2,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		// If corners are found in both images.
		if(found1 && found2) 
		{

			// just do it once.
			if(this->imageSize == Size())
			{
				 this->imageSize = img1.size();
				 std::cerr << "FOUND IMAGE SIZe=" << 	this->imageSize << "\n";
			}

			std::cerr << "Done " << i << "/"  << num_imgs << "." << "\n";

			cv::Mat gray1,gray2;
			cvtColor(img1, gray1, CV_BGR2GRAY);
			cvtColor(img2, gray2, CV_BGR2GRAY);
	
			// store in global vector for display later. (after rectification and undistortion process)
			goodGrayImageList.push_back(gray1.clone());
			goodGrayImageList.push_back(gray2.clone());

			// Corner sub-pixel refinement.
			cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(img1, chessBoardDimension, corners1, found1);

			
			cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(img2, chessBoardDimension, corners2, found2);

			//std::cerr << "Found corners for image pair i = " << i << endl;
			imagePoints1.push_back(corners1);
			imagePoints2.push_back(corners2);

			// "i" is incremented only when corners are found in both images.
			++i;
		}
		
		// show the image pair regardless of whether corners are found.
		imshow(left_camera_window, img1);
		imshow(right_camera_window, img2);
	
    	char key = (char)waitKey(wait_key_delay);
			
		// 27 is ascii for escape key.
		if(key == 'q' || key == esc_ascii)
		{
			return false;	 
		}
		
	}

	// Release resources.
	cv::destroyAllWindows();
	vid1.release(); vid2.release();


	// once all corners have been stored in buffer ...
	// put them into the globals 
	for (int i = 0; i < imagePoints1.size(); i++) 
	{
		std::vector< Point2f > v1, v2;
		for (int j = 0; j < imagePoints1[i].size(); ++j) 
		{
		  v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
		  v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
		}
		left_img_points.push_back(v1);
		right_img_points.push_back(v2);
	}


	return true;

}





//========================= Stereo Calibration Section ============================


// The imp function where most tasks are carried out.
// 1) Simply fetches the pre-known 3D object coordinate (Z = 0 always) of chessboard corners 
// 2) Then reads in the camera matrices internal parameters from files --> Make this optional later ?
// 3) Uses cv::stereoCalibrate() to find R and T between camera1 and camera2 + improves intrinsic params
// 4) If result of step 3) gives better/lower rms than before, then overwrite parameter in files.


int StereoCamera::runAndSaveStereoCalibration()
{	
	// fetch object/3D points
	std::vector<std::vector<Point3f> > objectPoints(1);
	calcKnownBoardCornerPositions(chessBoardDimension,calibSquareSize,objectPoints[0]);
	// can use either left or right imagepoint list to resize.
	objectPoints.resize(left_img_points.size(),objectPoints[0]);
	


	cv::Mat cameraMatrix1,distCoeffs1,cameraMatrix2, distCoeffs2;
	// rotation and translation between left and right camera.
	// Can think of the rotation vector as rotation required to 
	// convert camera 1 orientation into camera 2.
	cv::Mat R,T,E,F;



	cameraMatrix1 = this->left_camera.cameraMatrix.clone();
	distCoeffs1 =  this->left_camera.distCoeffs.clone();
	cameraMatrix2 = this->right_camera.cameraMatrix.clone();
	distCoeffs2 = this->right_camera.distCoeffs.clone();
	
	// Just in case single camera calibration has not been done.
	// It is recommended that single camera calibration be done since that 
	// would provide the distortion coefficients as well.
	//cameraMatrix1 = initCameraMatrix2D(objectPoints,left_img_points,imageSize,0);
	//cameraMatrix2 = initCameraMatrix2D(objectPoints,right_img_points,imageSize,0);


	int total_iterations =  600 ; //400
	int min_error_threshold = 1e-6;
	double rms = cv::stereoCalibrate(objectPoints,left_img_points,right_img_points,
																cameraMatrix1,distCoeffs1,
																cameraMatrix2,distCoeffs2,imageSize,
																R, T, E, F,
																
															    CALIB_ZERO_TANGENT_DIST +
															    CALIB_USE_INTRINSIC_GUESS +
															    CALIB_RATIONAL_MODEL +
															    CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
																cv::TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, total_iterations ,min_error_threshold) );

	std::cerr  << "CameraMatrix1 vector:\n" << cameraMatrix1 << "\n" ; 
	std::cerr  << "CameraMatrix2 vector:\n" << cameraMatrix2 << "\n" ; 
	

	std::cerr  << "Stereo calibration re-projection rms error = " << rms << "\n" ;	
	std::cerr  << "Translation vector:\n" << T << "\n" ; 


	// save intrinsic parameters,why after stereoCalibrate ? 
	// According to docs, it improves them
	// Only store results if better/lesser re-projection error than before
	bool overwrite = betterThanPreviousReprojectionError(this->extrinsics_file.c_str(),rms);
	if(overwrite)
	{	
		std::cerr << "Current rms is better hence overwriting!...\n";
		this->updateIntrinsicParams(cameraMatrix1,distCoeffs1,cameraMatrix2,distCoeffs2, true);
		this->updateExtrinsicParams(R,T,F,rms, true);
		
	}
	
	// Release resources
	cv::destroyAllWindows();

}


// ========================= Stereo Rectification Section =======================


// Rectifies images using cv::stereoRectify() and appends results to extrinsics file
// Stereo Rectification is done to ease the stereo Correspondence problem.
// The output of rectification is a rotation matrix for each camera


// Uses cv::stereoRectify(R,T,etc) -> R1,T1,P1,P2 + save output as extrinsic params in xml file

int StereoCamera::rectifyImages(bool display/*=false*/)
{
	std::cerr << "Rectifying images..." << "\n";
	cv::Mat cameraMatrix1 = this->left_camera.cameraMatrix.clone();
	cv::Mat distCoeffs1 = this->left_camera.distCoeffs.clone();

	cv::Mat cameraMatrix2 = this->right_camera.cameraMatrix.clone();
	cv::Mat distCoeffs2 = this->right_camera.distCoeffs.clone();

	cv::Mat R = this->R.clone();
	cv::Mat T = this->T.clone();
	cv::Mat F = this->F.clone();


	cv::Mat R1, R2, P1, P2, Q; // Rectification matrices - Output of cv::stereoRectify()
	Rect validRoi[2];

	cv::Size imageSize(640,480);
	// Image rectification 
	cv::stereoRectify(cameraMatrix1, distCoeffs1,
	              cameraMatrix2, distCoeffs2,
	              imageSize, R, T, R1, R2, P1, P2, Q,
	              CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	

	bool appendToExtrinsicsFile = true;
	this->updateRectificationParams( R1, P1, R2, P2, Q,appendToExtrinsicsFile);
	

	// Display rectified images
	if(display)
	{
		
		// This is used later to display images side by side or one below the other.
		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));


		//compute maps for cv::remap()
		cv::Mat rmap[2][2];
		cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);


		// Create a (coloured)canvas to display rectified images.
		cv::Mat canvas;
		double sf;
		int w, h;
		if( !isVerticalStereo )
		{
			sf = 600./MAX(imageSize.width, imageSize.height);
			w = cvRound(imageSize.width*sf);
			h = cvRound(imageSize.height*sf);
			canvas.create(h, w*2, CV_8UC3);
		}
		else
		{
			sf = 300./MAX(imageSize.width, imageSize.height);
			w = cvRound(imageSize.width*sf);
			h = cvRound(imageSize.height*sf);
			canvas.create(h*2, w, CV_8UC3);
		}


		cv::RNG rng(0);

		std::cerr << "Displaying epipolar lines on Stereo Calibration images to verify Rectification...\n";
		
		std::vector< Point2f > observedCorners1, observedCorners2;

		
		cv::VideoCapture vid1(1);
		cv::VideoCapture vid2(2);
		if(vid1.isOpened() && vid2.isOpened())
		{
			cv::Mat img1, img2;

			while(1)
			{
				if(vid1.read(img1) && vid2.read(img2))
				{

					if( !img1.empty() && !img2.empty() )
					{
						img2 = reduceBrightness(img2);

						cv::imshow("original LEFT", img1);
						cv::imshow("original RIGHT", img2);


						

						// 1) Undistort Images.
						cv::Mat undistorted_img1,undistorted_img2;
						cv::undistort(img1, undistorted_img1, cameraMatrix1, distCoeffs1);
						cv::undistort(img2, undistorted_img2, cameraMatrix2, distCoeffs2);

						// 2) Rectify them

						cv::Mat rectified_img1, cimg1;

						// The remap function works as follows - where (x,y) is a pixel coordinate
						// dest(x,y) = src(rmap[0][0]*(x,y), rmap[0][1]*(x,y)) ??!
						// if rmap[0][0/1]*(x,y) results in non-integer coordinates, then pixel value is interpolated(bilinear).
						cv::remap(img1, rectified_img1, rmap[0][0], rmap[0][1], INTER_LINEAR);
						 // done so that lines (see below) can be displayed in colour
						//cv::cvtColor(rectified_img1, cimg1, COLOR_GRAY2BGR);
						cv::Mat canvasPart1 = !isVerticalStereo ? canvas(Rect(0, 0, w, h)) : canvas(Rect(0, 0, w, h));
						cv::resize(rectified_img1, canvasPart1, canvasPart1.size(), 0, 0, INTER_AREA);

						cv::Mat rectified_img2, cimg2;

						cv::remap(img2, rectified_img2, rmap[1][0], rmap[1][1], INTER_LINEAR);
						//cv::cvtColor(rectified_img2, cimg2, COLOR_GRAY2BGR);
						cv::Mat canvasPart2 = !isVerticalStereo ? canvas(Rect(w, 0, w, h)) : canvas(Rect(0, h, w, h));
						resize(rectified_img2, canvasPart2, canvasPart2.size(), 0, 0, INTER_AREA);

						
						/*
						//===== corners ======
						bool foundBoth = this->findCorrespondingFeaturesBothImages(rectified_img1, rectified_img2, observedCorners1, observedCorners2, true, true);
						
						if(foundBoth)
						{
							//===== Compute and Display Epipolar lines ======
						
							// Remember epipolar lines are simply [a,b,c] in homogenous coordinates.
							std::vector<Vec3f> epilines1,epilines2;
							cv::computeCorrespondEpilines(observedCorners1, 1, F, epilines1); //Index starts with 1
							cv::computeCorrespondEpilines(observedCorners2, 2, F, epilines2);

					
							// select random color from [0,256)
							cv::Scalar line_color(rng(256),rng(256),rng(256));
					
							// For some of the detected points in a pair of images, display epipolar lines.
							for(int i = 0 , len = observedCorners1.size() ; i < len ; i+=1)
							{
								// To draw a line, specify 2 end points.
								// The 2 end points below are obtained by simply substituting x = 0 and x = img1/2.cols.
								// in ax+by+c=0 and then obtaining corresponding y.
								cv::line(canvasPart1,cv::Point(0,-epilines2[i][2]/epilines2[i][1]),
											cv::Point(rectified_img1.cols,-(epilines2[i][2]+epilines2[i][0]*rectified_img1.cols)/epilines2[i][1]),line_color);

								cv::line(canvasPart2,cv::Point(0,-epilines1[i][2]/epilines1[i][1]),
											cv::Point(rectified_img2.cols,-(epilines1[i][2]+epilines1[i][0]*rectified_img2.cols)/epilines1[i][1]),line_color);
				
								// To check if epipolar lines are passing through (corresponding) points, display points on image as well.
								cv::circle(canvasPart1, observedCorners1[i], 3, line_color, -1, CV_AA);
								cv::circle(canvasPart2, observedCorners2[i], 3, line_color, -1, CV_AA);

							}
						}
						*/

						if( !isVerticalStereo )
						{
			            	for( int j = 0; j < canvas.rows; j += 16 )
			                	line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
						}

			        	else
			        	{
			            	for( int j = 0; j < canvas.cols; j += 16 )
			                	line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
			        	}
									
					

			        }

					else
					{ 
						std::cerr << "One of the iamge pairs is empty ! " <<  "\n"; 
					}

					
					cv::imshow("rectified", canvas);

					char c = (char)waitKey(0);
					if( c == esc_ascii || c == 'q' || c == 'Q' )
					{
						break;
					}

					// save images 
					
						cv::imwrite("rectified_images.png",canvas);
					

				}

				
			}

			cv::destroyAllWindows();


		}
	}
	
}





// ================== Stereo Correspondence using SURF + Flann Matcher ==========

bool StereoCamera::findCorrespondingFeaturesBothImages(cv::Mat img1,cv::Mat img2, std::vector< Point2f >& observedCorners1, std::vector< Point2f >& observedCorners2, bool corner_refine/*=true*/, bool display/*=false*/)
{
	std::string matches_window_name = "Good Matches";
	cv::namedWindow(matches_window_name, cv::WINDOW_NORMAL);
	
	//cv::moveWindow(matches_window_name, 400,300);
	//cv::resizeWindow(matches_window_name,800,400);

	bool foundBoth = false; 

	//-- Step 1: Feature Detector + Descriptor ( SURF )
	// only features whose hessian is larger than threshold are accepted.
	// increase this if you want fewer detected keypoints.
	int minHessian = 300 ;
	Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
	detector->setHessianThreshold(minHessian);

	std::vector<KeyPoint> keypoints_1, keypoints_2;
	cv::Mat descriptors_1, descriptors_2;
	
	// Detect keypoints and compute descriptors.
	detector->detectAndCompute( img1, Mat(), keypoints_1, descriptors_1 );
	detector->detectAndCompute( img2, Mat(), keypoints_2, descriptors_2 );

	//-- Step 2: Matching descriptor vectors using FLANN matcher
	cv::FlannBasedMatcher matcher;
	std::vector< DMatch > matches; // DMatch is openCV class that holds matched descriptors

	if(descriptors_1.empty() || descriptors_2.empty())
	{
		std::cerr << "No features were detected in one/both images !\n";
	}
	else
	{
		matcher.match( descriptors_1, descriptors_2, matches );
	
	
		double max_dist = 0; double min_dist = 100;
		//-- Quick calculation of max and min distances between keypoints
		for( int i = 0; i < descriptors_1.rows; ++i )
		{ double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}
	
		// distance below refers to distance between descritors
		// Filter out "good" matches (i.e. whose distance is less than 2*min_dist,
		// or a small arbitary value ( 0.02 ) in the event that min_dist is very
		// small)
		//-- PS.- radiusMatch can also be used here.
		std::vector< DMatch > good_matches;
		for( int i = 0; i < descriptors_1.rows; i++ )
		{	
			 if( matches[i].distance <= max(2*min_dist, 0.02) )
			{ good_matches.push_back( matches[i]); }
		}

	
		for(int i = 0 , len = good_matches.size() ; i < len ; ++i)
		{
			// Here queryIdx indexes into keyPoints1 since match() was called in specific order(above).
			observedCorners1.push_back(keypoints_1[good_matches[i].queryIdx].pt);
			observedCorners2.push_back(keypoints_2[good_matches[i].trainIdx].pt);	
		}

		// ==== Subpixel corner-refinement ===
		if(corner_refine)
		{
			//std::cerr << "started refining ..." << "\n";
			
			cv::Mat gray1,gray2;
			cvtColor(img1, gray1, CV_BGR2GRAY);
			cvtColor(img2, gray2, CV_BGR2GRAY);

			cv::cornerSubPix(gray1, observedCorners1, cv::Size(5, 5), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

			cv::cornerSubPix(gray2, observedCorners2, cv::Size(5, 5), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				
			
		}

		if(display)
		{
			std::cerr << "Now displaying ..." << "\n";
			cv::Mat img_matches;
			drawMatches( img1, keypoints_1, img2, keypoints_2,
				   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
			//-- Show detected matches
			cv::imshow( matches_window_name, img_matches );
		
		}

		foundBoth = !good_matches.empty();
	}

	return foundBoth;

}



//======================== Triangulation Section ===========================



cv::Mat StereoCamera::doTriangulate_SingleFrame(cv::Mat img1, cv::Mat img2, bool display/*=false*/)
{
	std::stringstream ss;  // used during display.


	// needed for undistorting and triangulting feature points.
	cv::Mat cameraMatrix1 = this->left_camera.cameraMatrix;
	cv::Mat distCoeffs1 = this->left_camera.distCoeffs;
	cv::Mat R1 = this->R1;
	cv::Mat P1 = this->P1;
	cv::Mat cameraMatrix2 = this->right_camera.cameraMatrix;
	cv::Mat distCoeffs2 = this->right_camera.distCoeffs;
	cv::Mat R2 = this->R2;
	cv::Mat P2 = this->P2;
	

	 // For each image, find corresponding features, undistort the found features and then triangulate
	 std::vector< Point2f > observedCorners1, observedCorners2, undistortedCorners1, undistortedCorners2;

		
	bool foundBoth = this->findCorrespondingFeaturesBothImages(img1,img2,observedCorners1,observedCorners2,true,true);
		
		if(foundBoth && !observedCorners1.empty() && !observedCorners2.empty()  ) 
		{
			//========== Undistort and triangulate ============

			//convertPoint2fTo2ChannelMatrix(observedCorners1) ?? Do I need to do this to convert input into acceptable format ( according to doc ) ?

			// From the API doc -
			// cv::undistortPoints() first converts observed features to "normalized" ( independent of camera )
			// by applying inverse of camera matrix. Then it undistorts it using cv::undistort().
			// If P matrix is given, it converts to 3D coordinates and does a projection back to 2D coordinates.

			// Note - For 3D objects it does not reconstruct 3D coordinates ( but how does it know that input image	
			// contains 3D object and not a planar one like a chessboard ?)
			cv::undistortPoints(observedCorners1,undistortedCorners1,cameraMatrix1,distCoeffs1,R1,P1);
			cv::undistortPoints(observedCorners2,undistortedCorners2,cameraMatrix2,distCoeffs2,R2,P2);

			//std::cerr << "After undistort" << "\n";
			//std::cerr << "Observed corners 1 = \n" << observedCorners1 << "\n";
			//std::cerr << "uUndistorted corners 1 = \n" << undistortedCorners1 << "\n";

			//  cv::Mat projMat1,projMat2 ; projMat1 = computeProjectionMatrix(cameraMatrix1,R,T); projMat2 = computeProjectionMatrix(cameraMatrix2,R,T);
			cv::Mat pnts4D(1,undistortedCorners2.size(),CV_64FC4); // For output of cv::triangulatePoints()			
			cv::triangulatePoints(P1,P2,undistortedCorners1,undistortedCorners2,pnts4D);
		
			// since output of triangulation is 1 X N 4-channel array ( homogenous coordinates )
			// we convert it to a 4 X N 1-channel Matrix using cv::merge().
			cv::Mat multiChannelMat = convertToMultiChannel(pnts4D);
			
			//convert output to 3D cartesian coordinates (x,y,z)
			cv::Mat pnts3D(1,undistortedCorners1.size(),CV_64FC3);
			cv::convertPointsFromHomogeneous(multiChannelMat,pnts3D);
				
			if(display)
			{
				// Display (x,y,z) coordinates on the image(s)
				// Should I undistort image and display coordinates on undistorted image ?
				for(unsigned i=0, len = observedCorners1.size(); i < len ; i+=1)
				{
					ss << "( " << pnts3D.at<float>(i,0) <<" , " << pnts3D.at<float>(i,1) << " , " << pnts3D.at<float>(i,2) <<" )" ;
				
					cv::circle(img1, observedCorners1[i], 3, GREEN, -1, CV_AA);
					cv::putText(img1,ss.str(),observedCorners1[i], cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, RED);
					//cv::putText(img2,ss.str(),observedCorners2[i], cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, RED);
				

					ss.str(""); //clear the stringstream buffer
				}
				
			}
			
			return pnts3D;

		}

		else
		{
			std::cerr << "No corresponding features found!\n";
			cv::Mat empty;
			return empty;
		}
	

	
} 



// Input - Main Inputs are extrinsic parameters of a stereo camera setup. 
// Note - the extrinsic are only valid for a particular stereo camera setup
// 		  and hence If the cameras' relative orientation changes, then the extrinsics
// 		  will change too and hence new extrinsics will need to be found before triangulation.
int StereoCamera::doTriangulate()
{
	std::stringstream ss;  // used during display.

	std::cerr << "Assuming stereo Rectification has been (since P1, R1 ,etc are required..) ...\n";
	std::cerr << "Reading Intrinsics and Extrinsic parameters of stereo camera setup ...\n";
		
	// needed for undistorting and triangulting feature points.
	cv::Mat cameraMatrix1 = this->left_camera.cameraMatrix.clone();
	cv::Mat distCoeffs1 = this->left_camera.distCoeffs.clone();
	cv::Mat R1 = this->R1.clone();
	cv::Mat P1 = this->P1.clone();
	cv::Mat cameraMatrix2 = this->right_camera.cameraMatrix.clone();
	cv::Mat distCoeffs2 = this->right_camera.distCoeffs.clone();
	cv::Mat R2 = this->R2.clone();
	cv::Mat P2 = this->P2.clone();
	
	
	std::cerr << "Entered triangulation Section ..." << "\n";
	cv::VideoCapture vid1(this->left_camera.id);
	cv::VideoCapture vid2(this->right_camera.id);
	
	std::string left_camera_window = "Triangle - Left camera view";
	std::string right_camera_window = "Triangle - Right camera view";

	cv::namedWindow(left_camera_window);
	cv::namedWindow(right_camera_window);
	cv::moveWindow(left_camera_window, 20,20);
	cv::moveWindow(right_camera_window, 700,20);
	//cv::resizeWindow(left_camera_window, 500,500);
	//cv::resizeWindow(right_camera_window, 500,500);

	if(vid1.isOpened() && vid2.isOpened() )
	{
		// For each image, find corresponding features and then triangulate
		while(true)
		{
			cv::Mat img1,img2;
			vector< Point2f > observedCorners1, observedCorners2, undistortedCorners1, undistortedCorners2;

			img2 = reduceBrightness(img2);

			if(!vid1.read(img1) || !vid2.read(img2) )
			{	
				std::cerr << "Can't read images from either of the cameras !!\n";
				break;
			}
			
			//Rectify and undistorted images.

			bool foundBoth = this->findCorrespondingFeaturesBothImages(img1,img2,observedCorners1,observedCorners2,true,true);
			
			if(foundBoth) 
			{
				//========== Undistort and triangulate ============
	
				//convertPoint2fTo2ChannelMatrix(observedCorners1) ?? Do I need to do this to convert input into acceptable format ( according to doc ) ?

				// From the API doc -
				// cv::undistortPoints() first converts observed features to "normalized" ( independent of camera )
				// by applying inverse of camera matrix. Then it undistorts it using cv::undistort().
				// If P matrix is given, it converts to 3D coordinates and does a projection back to 2D coordinates.

				// Note - For 3D objects it does not reconstruct 3D coordinates ( but how does it know that input image	
				// contains 3D object and not a planar one like a chessboard ?)
				cv::undistortPoints(observedCorners1,undistortedCorners1,cameraMatrix1,distCoeffs1,R1,P1);
				cv::undistortPoints(observedCorners2,undistortedCorners2,cameraMatrix2,distCoeffs2,R2,P2);

				//std::cerr << "After undistort" << "\n";
				//std::cerr << "Observed corners 1 = \n" << observedCorners1 << "\n";
				//std::cerr << "uUndistorted corners 1 = \n" << undistortedCorners1 << "\n";

				//  cv::Mat projMat1,projMat2 ; projMat1 = computeProjectionMatrix(cameraMatrix1,R,T); projMat2 = computeProjectionMatrix(cameraMatrix2,R,T);
				cv::Mat pnts4D(1,undistortedCorners2.size(),CV_64FC4); // For output of cv::triangulatePoints()			
				cv::triangulatePoints(P1,P2,undistortedCorners1,undistortedCorners2,pnts4D);
			
				// since output of triangulation is 1 X N 4-channel array ( homogenous coordinates )
				// we convert it to a 4 X N 1-channel Matrix using cv::merge().
				cv::Mat multiChannelMat = convertToMultiChannel(pnts4D);
				
				//convert output to 3D cartesian coordinates (x,y,z)
				cv::Mat pnts3D(1,undistortedCorners1.size(),CV_64FC3);
				cv::convertPointsFromHomogeneous(multiChannelMat,pnts3D);
				
				/*	
				// since y-axis is inverted.
				int rows = pnts3D.rows;
				int cols = pnts3D.cols;
				for(int y = 0 ; y < rows; ++y)
				{
					for(int x = 0 ;x < cols ; ++x)
					{
						pnts3D.at<Vec3b>(y, x)[1] = -1 * pnts3D.at<Vec3b>(y, x)[1] ;
					}				}
				}
				*/

				// Display (x,y,z) coordinates on the image(s)
				for(unsigned i=0, len = observedCorners1.size(); i < len ; i+=1)
				{
					ss << "( " << pnts3D.at<float>(i,0) <<" , " << -1*pnts3D.at<float>(i,1) << " , " << pnts3D.at<float>(i,2) <<" )" ;
					
					//cout << "(x,y,z) =" << pnts3D.col(i) << "\n";
					cv::circle(img1, observedCorners1[i], 3, GREEN, -1, CV_AA);
					cv::putText(img1,ss.str(),observedCorners1[i], cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, RED);
					//cv::putText(img2,ss.str(),observedCorners2[i], cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, RED);
					

					ss.str(""); //clear the stringstream buffer
				}
				
			}
	
			else
			{
				std::cerr << "No corresponding features found!\n";
			}

			cv::imshow(left_camera_window, img1);
			cv::imshow(right_camera_window, img2);

			char key = (char)waitKey(20);
		
			if(key == 'q' || key == esc_ascii)
			{
				break;
			}

			else if(key == 's')
			{
				cv::imwrite("Triangulation.jpg",img1);
			}
	
		}

		cv::destroyAllWindows();
		vid1.release(); vid2.release();

	}

	else
	{
		std::cerr << "Can't open video device input Can't triangulate!!\n";
	}

	
	
} 












/*

// usage
// string ty =  type2str( M.type() );
//printf("Matrix: %s %dx%d \n", ty.c_str(), M.cols, M.rows );
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

*/










//======================= Disparity Section ==========================================



int checkParameters(const std::string& img1_filename, const std::string& img2_filename,const std::string& intrinsic_filename,const std::string& extrinsic_filename,const std::string& point_cloud_filename)
{
	
    if( img1_filename.empty() || img2_filename.empty() )
    {
        printf("Function parameter error: both left and right images must be specified\n");
        return -1;
    }

    if( (!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()) )
    {
        printf("Function parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }


    if( extrinsic_filename.empty() && !point_cloud_filename.empty() )
    {
        printf("Function parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }

}



enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
// variables needed by trackbar
// values assigned here are default values.
int algVal                 = STEREO_SGBM;
int SADWindowSizeVal       = algVal == STEREO_SGBM ? 3 : 9; // block dimensions (must be odd) 3-11 range.
int preFilterCapVal        = 63;
int minDisparityVal        = 0 ; 
int numberOfDisparitiesVal = 1;
int uniquenessRatioVal     = 10; // 5-15%
int speckleWindowSizeVal   = 100; // 50-200
int setSpeckleRangeVal     = 32;  // 1 or 2
int P1Val 				   = 0;
int P2Val 				   = 0;




//enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
// from $OPENCV_DIR/samples/cpp/stereo_match.cpp
int findDisparityAndDepth(const std::string& img1_filename, const std::string& img2_filename,
					 const std::string& intrinsic_filename,const std::string& extrinsic_filename,
					const std::string& disparity_file,const std::string& pointcloud_filename,float scale=1.0f)
{
	

	Ptr<StereoBM> bm = StereoBM::create(16,9);
	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
	
	
	if( checkParameters(img1_filename,img2_filename,intrinsic_filename ,extrinsic_filename, pointcloud_filename) == -1)
	{
		return -1;
	}


	// Stereo_BM only accepts grayscale images.
    int color_mode = algVal == STEREO_BM ? 0 : -1;
    Mat img1 = imread(img1_filename, color_mode);
    Mat img2 = imread(img2_filename, color_mode);

    if (img1.empty() || img2.empty())
    {
       printf("Function parameter error: could not load one of the input image files\n");
       return -1;
    }
   

    if (scale != 1.0f)
    {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(img1, temp1, Size(), scale, scale, method);
        img1 = temp1;
        resize(img2, temp2, Size(), scale, scale, method);
        img2 = temp2;
    }

    Size img_size = img1.size();

    Rect roi1, roi2;
    Mat Q;
	

	if( !intrinsic_filename.empty() )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", intrinsic_filename.c_str());
            return -1;
        }

        Mat M1, D1, M2, D2;
        fs["cameraMatrix1"] >> M1;
        fs["distCoeffs1"] >> D1;
        fs["cameraMatrix2"] >> M2;
        fs["distCoeffs2"] >> D2;

        M1 *= scale;
        M2 *= scale;

        fs.open(extrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", extrinsic_filename.c_str());
            return -1;
        }

        Mat R, T, R1, P1, R2, P2;
        fs["R"] >> R;
        fs["T"] >> T;

        stereoRectify(M1,D1,M2,D2,img_size,R,T,R1,R2,P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

        img1 = img1r;
        img2 = img2r;
    }
   
  

	// variables needed by trackbar
	
	sgbm->setPreFilterCap(preFilterCapVal);
	int sgbmWinSize = (SADWindowSizeVal * 2) - 1;
	if (sgbmWinSize < 1 || sgbmWinSize % 2 != 1)
    {
        printf("The block size (--blocksize=<...>) must be a positive odd number\n");
        return -1;
    }

    sgbm->setBlockSize(SADWindowSizeVal);

    int cn = img1.channels();
    //sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    //sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    //sgbm->setMinDisparity(minDisparityVal);


    sgbm->setP1(P1Val);
    sgbm->setP2(P2Val);

    //minDisparityVal = minDisparityVal -50;
    sgbm->setMinDisparity(minDisparityVal);

    int numberOfDisparitiesVal = 16* (numberOfDisparitiesVal+1);
    if ( numberOfDisparitiesVal < 1 || numberOfDisparitiesVal % 16 != 0 )
    {
        printf("Function parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
        return -1;
    }

    sgbm->setNumDisparities(numberOfDisparitiesVal);
    sgbm->setUniquenessRatio(uniquenessRatioVal);
    sgbm->setSpeckleWindowSize(speckleWindowSizeVal);
    sgbm->setSpeckleRange(setSpeckleRangeVal);
    sgbm->setDisp12MaxDiff(1);

   
    if(algVal==1)
        sgbm->setMode(StereoSGBM::MODE_SGBM);
    else if(algVal==2)
    	sgbm->setMode(StereoSGBM::MODE_HH);
    else if(algVal==3)
        sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);


	 cv::Mat disp, disp8;

   // calculate time required to compute disparity using one of the BM algorithms
	int64 t = getTickCount();
   
    	sgbm->compute(img1, img2, disp);
    t = getTickCount() - t;
    //fprintf(stderr,"Time elapsed: %fms\n", t*1000/getTickFrequency());
	
	disp.convertTo(disp8, CV_8U);

	//display 
	namedWindow("left", 1);
    imshow("left", img1);
    namedWindow("right", 1);
    imshow("right", img2);
    namedWindow("disparity", 0);
    imshow("disparity", disp8);
    char ch =waitKey(200);
    /*
    fprintf(stderr,"Please re-tune parameters if needed and/or press a key to continue ...\n");
    char ch =waitKey(0);
	if(ch == 'q')
	{
		return -1;
	}
    printf("\n");
	*/

	/*

	if(!disparity_file.empty()) 
	{
		cv::imwrite(disparity_file, disp8); 
	}

	if(!point_cloud_filename.empty())
    {
        printf("storing the point cloud...");
        fflush(stdout);
        Mat xyz;
        cv::reprojectImageTo3D(disp, xyz, Q, true);
        saveXYZ(point_cloud_filename.c_str(), xyz);
        printf("\n");
    }

	*/

    return 0;

}



void createStereoTrackBars(const char* trackBarWindowName) 
{

	createTrackbar("alg",trackBarWindowName,&algVal,4,0);
	createTrackbar("SADWindowSize (convereted to odd)",trackBarWindowName,&SADWindowSizeVal,15,0) ;
	createTrackbar("preFilterCap",trackBarWindowName,&preFilterCapVal,100,0) ;
	createTrackbar("minDisparity (-50)",trackBarWindowName,&minDisparityVal,120,0) ;
	createTrackbar("numberOfDisparities( (n+1)*16 )",trackBarWindowName,&numberOfDisparitiesVal,12,0) ;
	createTrackbar("uniquenessRatio(5 to 15)",trackBarWindowName,&uniquenessRatioVal,16,0) ;
	createTrackbar("speckleWindowSize(50-200)",trackBarWindowName,&speckleWindowSizeVal,200,0) ;
	createTrackbar("setSpeckleRange",trackBarWindowName,&setSpeckleRangeVal,5,0) ;
	createTrackbar("P1",trackBarWindowName,&P1Val,800,0) ;
	createTrackbar("P2 (>P1)",trackBarWindowName,&P2Val,3000,0) ;
}



void StereoCamera::doDisparity(const char* intrinsics_file,const char* extrinsics_file)
{
	std::string pointcloud_file = "point_cloud.txt"; 
	std::string disparity_file = "disparity_img.jpg";
	std::string left_img_file = "left_img.jpg";
	std::string right_img_file = "right_img.jpg";

	// Create a trackbar for parameter adjustment	
	const char* trackBarWindowName = "Stereo_TrackBar";
	cv::namedWindow(trackBarWindowName);
	createStereoTrackBars(trackBarWindowName);
	
	// temporary buffers
	cv::VideoCapture vid1(1), vid2(2);
	cv::Mat left_img , right_img; 

	if(vid1.isOpened() && vid2.isOpened())
	{
		while(1)
		{
			if(vid1.read(left_img) && vid2.read(right_img))
			{
				cv::imwrite(left_img_file,left_img);
				cv::imwrite(right_img_file,right_img);

				numberOfDisparitiesVal = ((left_img.size().width/8) + 15) & -16;
				
				if(findDisparityAndDepth(left_img_file, right_img_file, intrinsics_file, extrinsics_file,
									disparity_file,pointcloud_file) == -1 )
				{
					break;		
				}
		
			}
			
		}
	}

}

