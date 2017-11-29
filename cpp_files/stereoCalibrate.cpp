#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>


#include "/home/malhar/opencv-3.3.0/apps/interactive-calibration/rotationConverters.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <cmath>

#include <opencv2/calib3d.hpp>

#define CALIB_PI 3.14159265358979323846
#define CALIB_PI_2 1.57079632679489661923
#define FONT_SIZE 0.5



using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
// variables needed by trackbar
int algVal                 = STEREO_SGBM;
int SADWindowSizeVal       = algVal == STEREO_SGBM ? 3 : 9;
int preFilterCapVal        = 63;
int minDisparityVal        = 0 ; 
int numberOfDisparitiesVal = 16;
int uniquenessRatioVal     = 10;
int speckleWindowSizeVal   = 100;
int setSpeckleRangeVal     = 32;


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



// A few globals for ease
const float calibSquareSize = 0.02578f; //metres
const Size chessBoardDimension(6,9); //width = 6 , height = 9 
int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
const cv::Scalar RED(0,0,255), GREEN(0,255,0); 
vector< vector< Point2f > > left_img_points, right_img_points; //stores all corners
Size imageSize;

int wait_key_delay = 1000;
std::vector<Mat> goodGrayImageList; // list of "pair" of images where features/corners were detcted

// Function prototypes
bool load_image_points(int num_imgs) ;




//==================== Helper functions ===================

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
	cout << "Size of matrix (row x col )= [" << mat.size().height << " x " << mat.size().width <<"]"  << "\n";
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


//========================== Loading images and detecting features ===============================

// This function only loads images taken from stereo cameras.
// In order to use it, images from both cameras must be present.
// Also note that the output is stored in a global vector for convenience purpse
bool load_image_points(int num_imgs) 
{
	vector<vector<Point2f> > imagePoints1,imagePoints2;
	VideoCapture vid1(1);
	VideoCapture vid2(2);
	
	if(!vid1.isOpened() || !vid2.isOpened() )
	{
		return false;
	}


	cout << "Reading input images from video stream ...." << "\n";
	// loop over each pair of left and right images.
	for(int i=0;;)
	{
		if(i >= num_imgs)
		{
			break ;
		}
	
		Mat img1,img2;
		vector< Point2f > corners1, corners2;

		if(!vid1.read(img1) || !vid2.read(img2) )
		{
			return false;
		}
		
	
		bool found1 = false, found2 = false;

		found1 = cv::findChessboardCorners(img1, chessBoardDimension, corners1,
		CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		found2 = cv::findChessboardCorners(img2, chessBoardDimension, corners2,
		CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		if (found1 && found2) 
		{
			// just do it once.
			if(imageSize == Size())
			{
				 imageSize = img1.size();
				std::cout << "FOUND IMAGE SIZ=" << 	imageSize << "\n";
			}

			
			Mat gray1,gray2;
			cvtColor(img1, gray1, CV_BGR2GRAY);
			cvtColor(img2, gray2, CV_BGR2GRAY);
	
			// store in global vector for display later. ( after rectification and undistortion process)
			goodGrayImageList.push_back(gray1.clone());
			goodGrayImageList.push_back(gray2.clone());

			cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(img1, chessBoardDimension, corners1, found1);

			cvtColor(img2, gray2, CV_BGR2GRAY);
			cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(img2, chessBoardDimension, corners2, found2);

			cout << "Found corners for i = " << i << endl;
			imagePoints1.push_back(corners1);
			imagePoints2.push_back(corners2);

			
			
			i++;
		}
		
		imshow("Left camera view", img1);
		imshow("Right camera view", img2);
	
        char key = (char)waitKey(wait_key_delay);
			
		if(key == 'q' || key == 27)
		{
			return false;	
		}
		
	}
	cv::destroyAllWindows();
	vid1.release(); vid2.release();


	// once all corners have been stored in buffer ...
	// put them into the globals 
	for (int i = 0; i < imagePoints1.size(); i++) 
	{
		vector< Point2f > v1, v2;
		for (int j = 0; j < imagePoints1[i].size(); j++) 
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

// save camera matrix and distortion coeffs.
void saveIntrinsicParameters(const char* intrinsics_file, Mat cameraMatrix1,Mat cameraMatrix2,Mat distCoeffs1, Mat distCoeffs2)
{
    FileStorage fs(intrinsics_file, FileStorage::WRITE);
	
	if( fs.isOpened() )
    {
        fs << "cameraMatrix1" << cameraMatrix1 << "distCoeffs1" << distCoeffs1 <<
            "cameraMatrix2" << cameraMatrix2 << "distCoeffs2" << distCoeffs2;
        fs.release();
    }
    else
	{
        cout << "Error: can not save the intrinsic parameters ( File cannot be opened )\n";
	}

}





// Note - Returns true if input file(extrinsics) is not having a reprojection entry field. ( In which case it will possibly be overwritten
// by whoever is using this function.)
bool betterThanPreviousReprojectionError(const char* extrinsics_file, double currentReprojectionError)
{
	bool betterThanPrevious = false;

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
			std::cout << "Emoty Node! (file might be overwritten)\n";
		}
	
		fs.release();
	}

	return betterThanPrevious;
}




// save relative camera pose...
void saveExtrinsicParameters(const char* extrinsics_file,cv::Mat R, cv::Mat T,cv::Mat F,double rms)
{
	cv::Mat rotVector_R ;
	myRot2Euler(R,rotVector_R);  // convert to more comprehensible form	
	
	FileStorage fs(extrinsics_file, FileStorage::WRITE);
	
    if( fs.isOpened() )
    {
        fs << "reprojError" << rms << "R" << R << "Euler_R" << rotVector_R << "T" << T << "F" << F  ;
        fs.release();
    }
    else
	{
        cout << "Error: can not save the extrinsic parameters after stereo Calibration\n";
	}

}



// Overloaded : save( by "appending" to extrinsics) rectification transformation matrices...
// What if Results already exist ?
// Since openCV doesnt allow duplicate keys in XML file ( and doesn't overwrite existing keys),
// and we wish to append results, we need to copy required existing file content and write it back.
void saveExtrinsicParameters(const char* extrinsics_file,Mat R1, Mat R2,Mat P1, Mat P2, Mat Q)
{
	cv::Mat R,rotVector_R,T,F;
	double reprojError;
	
	// read results ( only to copy then back ...)
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

	cv::Mat rotVector_R1 ;
	cv::Mat rotVector_R2 ;

	// convert to more comprehensible form	
		
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
        cout << "Error: can not save the extrinsic parameters after Stereo Rectification\n";
	}

}



void displayNormalLines(cv::Mat canvas,bool isVerticalStereo)
{
	if( !isVerticalStereo )
	{
	    for( int j = 0; j < canvas.rows; j += 16 )
	        line(canvas, Point(0, j), Point(canvas.cols, j), GREEN, 1, 8);
	}
	else
	{
	    for( int j = 0; j < canvas.cols; j += 16 )
	        line(canvas, Point(j, 0), Point(j, canvas.rows), GREEN, 1, 8);
	}

}
			


// Rectifies images using cv::stereoRectify() and appends results to extrinsics file
int rectifyImages(const char* intrinsics_file,const char* extrinsics_file, bool display=false)
{
	std::cout << "Rectifying images..." << "\n";
	cv::Mat cameraMatrix1, distCoeffs1;
	cv::Mat cameraMatrix2, distCoeffs2;
	cv::Mat R,T,F;

	// Read some parameters from the input files
	FileStorage fs1;
	fs1.open(intrinsics_file, FileStorage::READ);	

	if(!fs1.isOpened())
	{
		cout << "Please supply LEFT camera internal calibration parameters! "<< "\n";
		return -1;
	}
	
	fs1["cameraMatrix1"] >> cameraMatrix1 ;
	fs1["distCoeffs1"] >> distCoeffs1 ;
	fs1["cameraMatrix2"] >> cameraMatrix2 ;
	fs1["distCoeffs2"] >> distCoeffs2 ;

	fs1.release();  

	// Read some parameters from the input file
	FileStorage fs2;
	fs2.open(extrinsics_file, FileStorage::READ);	

	if(!fs2.isOpened())
	{
		std::cerr << "Please supply extrinsics! Required for rectification "<< "\n";
		return -1;
	}
	
	fs2["R"] >> R ;
	fs2["T"] >> T ;
	fs2["F"] >> F;
	
	fs2.release(); 


	cv::Mat R1, R2, P1, P2, Q; // Recitfication matrices - Output of cv::stereoRectify()
	Rect validRoi[2];

	// Image rectification 
	cv::stereoRectify(cameraMatrix1, distCoeffs1,
	             cameraMatrix2, distCoeffs2,
	              Size(640,480), R, T, R1, R2, P1, P2, Q,
	              CALIB_ZERO_DISPARITY, 1, Size(640,480), &validRoi[0], &validRoi[1]);

	// appends results to extrinsics file
	saveExtrinsicParameters(extrinsics_file,R1,R2,P1,P2,Q);
	
	std::cout << "Displaying rectified images..." << "\n";
	// Display rectified images
	if(display)
	{
		// This is used later to display images side by side or one below the other.
		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));


		//compute maps for cv::remap()
		 Mat rmap[2][2];
		initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);


		cv::Mat canvas; // it is coloured ( see later below )
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


		// iterate over pair of grayscale images
		for( int i = 0, len = goodGrayImageList.size()/2 ; i < len ; i++ )
		{ 
			Mat img1 = goodGrayImageList[i*2];
			cv::Mat img2 = goodGrayImageList[i*2+1];
			Mat rect_img1, cimg1;


			if( !img1.empty() && !img2.empty() )
			{
				// the remap function works as follows - where (x,y) is a pixel coordinate
				// dest(x,y) = src(rmap[0][0]*(x,y), rmap[0][1]*(x,y)) ??!
				remap(img1, rect_img1, rmap[0][0], rmap[0][1], INTER_LINEAR);
				cvtColor(rect_img1, cimg1, COLOR_GRAY2BGR); //this is done so that lines (see below ) can be displayed in colour
				cv::Mat canvasPart1 = !isVerticalStereo ? canvas(Rect(0, 0, w, h)) : canvas(Rect(0, 0, w, h));
				resize(cimg1, canvasPart1, canvasPart1.size(), 0, 0, INTER_AREA);

				
				cv::Mat rect_img2, cimg2;

				remap(img2, rect_img2, rmap[1][0], rmap[1][1], INTER_LINEAR);
				cvtColor(rect_img2, cimg2, COLOR_GRAY2BGR);
				cv::Mat canvasPart2 = !isVerticalStereo ? canvas(Rect(w, 0, w, h)) : canvas(Rect(0, h, w, h));
	
				resize(cimg2, canvasPart2, canvasPart2.size(), 0, 0, INTER_AREA);

		
				// display epipolar lines
				cv::Scalar line_color(rng(256),rng(256),rng(256));

				std::vector< Point2f > undistortedCorners1, undistortedCorners2;

				cv::undistortPoints(left_img_points[i],undistortedCorners1,cameraMatrix1,distCoeffs1,R1,P1);
				cv::undistortPoints(right_img_points[i],undistortedCorners2,cameraMatrix2,distCoeffs2,R2,P2);

				std::vector<Vec3f> epilines1,epilines2;
				cv::computeCorrespondEpilines(undistortedCorners1, 1, F, epilines1); //Index starts with 1
				cv::computeCorrespondEpilines(undistortedCorners2, 2, F, epilines2);

				
				// For some of the detected point in the pair of images
				for(int i = 0 , len = undistortedCorners1.size() ; i < len ; i+=len/8)
				{
					cv::line(canvasPart1,cv::Point(0,-epilines2[i][2]/epilines2[i][1]),cv::Point(img1.cols,-(epilines2[i][2]+epilines2[i][0]*img1.cols)/epilines2[i][1]),line_color);
	
					cv::line(canvasPart2,cv::Point(0,-epilines1[i][2]/epilines1[i][1]),
			  				cv::Point(img2.cols,-(epilines1[i][2]+epilines1[i][0]*img2.cols)/epilines1[i][1]),line_color);
			
					cv::circle(canvasPart1, undistortedCorners1[i], 3, line_color, -1, CV_AA);
					cv::circle(canvasPart1, undistortedCorners1[i], 3, line_color, -1, CV_AA);

				}
				

			}


			else{ std::cerr << "One of the iamge pairs is empty ! for i =" << i <<  "\n"; }

			
			

			
			cv::imshow("rectified", canvas);

			char c = (char)waitKey();
			if( c == 27 || c == 'q' || c == 'Q' )
			{
			   break;
			}
		}
	}
	
}




// The imp function where most tasks are carried out.
// 1) Simply fetches the pre-known 3D object coordinate ( Z = 0 always) of chessboard corners 
// 2) (optional)Then reads in the camera matrices internal parameters from files
// 3) Uses cv::stereoCalibrate() to find R and T between camera1 and camera2 + save improvised intrinsic params in xml file
// 4) Uses cv::stereoRectify(R,T,etc) -> R1,T1,P1,P2 + save output as extrinsic params in xml file

int RunAndSaveStereoCalibration(const char* leftCalibFile, const char* rightCalibFile,const char* intrinsics_file,const char* extrinsics_file)
{	
	// Create object points, can use either left or right imagepoint list to resize
	vector<vector<Point3f> > objectPoints(1);
	calcKnownBoardCornerPositions(chessBoardDimension,calibSquareSize,objectPoints[0]);
	objectPoints.resize(left_img_points.size(),objectPoints[0]);
	
	cv::Mat cameraMatrix1, distCoeffs1;
	cv::Mat cameraMatrix2, distCoeffs2;

	// Read some parameters from the input file
	FileStorage fs1;
	fs1.open(leftCalibFile, FileStorage::READ);	

	if(!fs1.isOpened())
	{
		cout << "Please supply LEFT camera internal calibration parameters! "<< "\n";
		return -1;
	}
	
	fs1["camera_matrix"] >> cameraMatrix1 ;
	fs1["distortion_coefficients"] >> distCoeffs1;

	fs1.release();  

	FileStorage fs2;
	fs2.open(rightCalibFile, FileStorage::READ);	

	if(!fs2.isOpened())
	{
		cout << "Please supply RIGHT camera internal calibration parameters! "<< "\n";
		return -1;
	}

	
	fs2["camera_matrix"] >> cameraMatrix2 ;
	fs2["distortion_coefficients"] >> distCoeffs2;

	fs2.release();  


	// rotation and translation between left and right camera.
	// Can think of the rotation vector as rotation required to 
	// convert camera 1 orientation into camera 2.
	cv::Mat R,T,E,F;
	
	double rms = cv::stereoCalibrate(objectPoints,left_img_points,right_img_points,
	cameraMatrix1,distCoeffs1,cameraMatrix2,distCoeffs2,imageSize,
	R, T, E, F,
	CV_CALIB_USE_INTRINSIC_GUESS,
	cv::TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );

	
	std::cerr  << "stereo calibration re-projection error = " << rms << "\n" ;	
	std::cerr  << "Translation vector" << "\n" << T << "\n" ; 

	

	// save intrinsic parameters,why after stereoCalibrate ? 
	//  According to docs, it improves them
	// Only store results if better re-projection error than befores
	bool better = betterThanPreviousReprojectionError(extrinsics_file,rms);
	if(better)
	{	
		std::cerr << "Current rms is better hence overwriting!..." << "\n";
		saveIntrinsicParameters(intrinsics_file,cameraMatrix1,cameraMatrix2,distCoeffs1,distCoeffs2);
		saveExtrinsicParameters(extrinsics_file,R,T,F,rms);
	}
	

	cv::destroyAllWindows();

}


//======================== Triangulation Section ===========================

// Find feature points in both input images.
bool foundCorrespondingFeaturesBothImages(cv::Mat img1,cv::Mat img2,vector< Point2f >& observedCorners1,vector< Point2f >& observedCorners2,bool corner_refine=true, bool display=false)
{

	bool foundBoth = false; 

	/*
		bool found1 = cv::findChessboardCorners(img1, chessBoardDimension, observedCorners1,
		CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		bool found2 = cv::findChessboardCorners(img2, chessBoardDimension, observedCorners2,
		CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);


		foundBoth = found1 && found2;
	*/
	

	//-- Step 1: Feature Detector + Descritor ( SURF )
	int minHessian = 400;
	Ptr<SURF> detector = SURF::create();
	detector->setHessianThreshold(minHessian);
	std::vector<KeyPoint> keypoints_1, keypoints_2;
	Mat descriptors_1, descriptors_2;
	detector->detectAndCompute( img1, Mat(), keypoints_1, descriptors_1 );
	detector->detectAndCompute( img2, Mat(), keypoints_2, descriptors_2 );

	//-- Step 2: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches; // DMatch is openCV class that holds matched descriptors

	if(descriptors_1.empty() || descriptors_2.empty())
	{
		std::cout << "One of the iamges does not have any features !\n";
	}
	else
	{
		matcher.match( descriptors_1, descriptors_2, matches );
	
	
	

		double max_dist = 0; double min_dist = 100;
		//-- Quick calculation of max and min distances between keypoints
		for( int i = 0; i < descriptors_1.rows; i++ )
		{ double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}
		//printf("-- Max dist : %f \n", max_dist );
		//printf("-- Min dist : %f \n", min_dist );
		//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
		//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
		//-- small)
		//-- PS.- radiusMatch can also be used here.
		std::vector< DMatch > good_matches;
		for( int i = 0; i < descriptors_1.rows; i++ )
		{	
			 if( matches[i].distance <= max(2*min_dist, 0.02) )
			{ good_matches.push_back( matches[i]); }
		}

	
		for(int i = 0 , len = good_matches.size() ; i < len ; ++i)
		{
			observedCorners1.push_back(keypoints_1[good_matches[i].queryIdx].pt);
			observedCorners2.push_back(keypoints_2[good_matches[i].trainIdx].pt);	
		}

		if(corner_refine)
		{
			cout << "started refining ..." << "\n";
			// Subpixel corner refinement
			cv::Mat gray1,gray2;
			cvtColor(img1, gray1, CV_BGR2GRAY);
			cvtColor(img2, gray2, CV_BGR2GRAY);

			cv::cornerSubPix(gray1, observedCorners1, cv::Size(5, 5), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

			cv::cornerSubPix(gray2, observedCorners2, cv::Size(5, 5), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				
			cout << "finished refining ..." << "\n";
		}

		if(display)
		{
			cv::Mat img_matches;
			drawMatches( img1, keypoints_1, img2, keypoints_2,
				   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
			//-- Show detected matches
			cv::imshow( "Good Matches", img_matches );
		
			/*
				// Size(5,5) is recommended (by docs) neighbourhood size used in corner refinement algorithm.
				// It is in-place, observedCorners1 is input and output.
	
				cv::drawChessboardCorners(img1, chessBoardDimension, observedCorners1, found1);
				cv::drawChessboardCorners(img2, chessBoardDimension, observedCorners2, found2);
			*/
		}

		foundBoth = !good_matches.empty();
	}

	return foundBoth;

}




// Input - Main Inputs are extrinsic parameters of a stereo camera setup. 
// Note - the extrinsic are only valid for a particular stereo camera setup
// 		  and hence If the cameras' relative orientation changes, then the extrinsics
// 		  will change too and hence new extrinsics will need to be found before triangulation.

//Note - intrinsics_file and extrinsics_file contain parameters for stereo cameras
int doTriangulate(const char* intrinsics_file,const char* extrinsics_file)
{
	std::stringstream ss;  // used during display.
	cv::Mat P1,R1,cameraMatrix1,distCoeffs1,P2, R2,cameraMatrix2,distCoeffs2;

	cout << "Assuming stereo REctification has been done ..." << "\n";
	cout << "Reading Intrinsics and Extrinsic parameters of stereo camera setup ..." << "\n";
		
	FileStorage fs1;
	fs1.open(intrinsics_file, FileStorage::READ);	

	FileStorage fs2;
	fs2.open(extrinsics_file, FileStorage::READ);	

	if(!fs1.isOpened() || !fs2.isOpened())
	{
		cout << "Could not open parameters file! "<< "\n";
		return -1;
	}

	fs1["cameraMatrix1"] >> cameraMatrix1 ;
	fs1["cameraMatrix2"] >> cameraMatrix2 ;
	fs1["distCoeffs1"] >> distCoeffs1 ;
	fs1["distCoeffs2"] >> distCoeffs2 ;

	fs2["P1"] >> P1;
	fs2["R1"] >> R1 ;
	fs2["P2"] >> P2;
	fs2["R2"] >> R2;

	fs1.release();  
	fs2.release();

	
	cout << "Entered triangulation Section ..." << "\n";
	VideoCapture vid1(1);
	VideoCapture vid2(2);
	
	
	int h_min = 0 , h_max = 9 ;
			int s_min = 207, s_max = 255;
			int v_min = 24 , v_max = 236;
	
	int h_min2 = 0 , h_max2 = 216 ;
			int s_min2 = 143, s_max2 = 255;
			int v_min2 = 218 , v_max2 = 255;
	if(vid1.isOpened() && vid2.isOpened() )
	{
		for(;;)
		{
			cv::Mat img1,img2;
			vector< Point2f > observedCorners1, observedCorners2, undistortedCorners1, undistortedCorners2;

			if(!vid1.read(img1) || !vid2.read(img2) )
			{	
				cout << "Can't read images from either of the cameras !!" << "\n";
				break;
			}
	
			
			// convert to HSV ?
			cv::Mat hsv_frame1, hsv_frame2 ;

			cv::cvtColor(img1,hsv_frame1,CV_BGR2HSV);
			cv::cvtColor(img2,hsv_frame2,CV_BGR2HSV);
				
			
			cv::inRange(hsv_frame1,Scalar(h_min,s_min,v_min),Scalar(h_max,s_max,v_max),hsv_frame1);
			cv::inRange(hsv_frame2,Scalar(h_min2,s_min2,v_min2),Scalar(h_max2,s_max2,v_max2),hsv_frame2);
			
			cvtColor(hsv_frame1, img1, CV_GRAY2BGR); 
			cvtColor(hsv_frame2, img2, CV_GRAY2BGR); 
			
			std::cout << "CONVERTED to HSV.... !!" << "\n";
			bool foundBoth = foundCorrespondingFeaturesBothImages(img1,img2,observedCorners1,observedCorners2,true,false);
			
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

				//cout << "After undistort" << "\n";
				//cout << "Observed corners 1 = \n" << observedCorners1 << "\n";
				//cout << "uUndistorted corners 1 = \n" << undistortedCorners1 << "\n";

				cv::Mat pnts4D(1,undistortedCorners2.size(),CV_64FC4); // For output of cv::triangulatePoints()
				//  cv::Mat projMat1,projMat2 ; projMat1 = computeProjectionMatrix(cameraMatrix1,R,T); projMat2 = computeProjectionMatrix(cameraMatrix2,R,T);
								
				
				cv::triangulatePoints(P1,P2,undistortedCorners1,undistortedCorners2,pnts4D);
			
				//cout << "The 4D points are = " << "\n" << pnts4D << "\n"; 

				// since output of triangulation is 1XN 4 channel array ( homogenous coordinates )
				// we convert it to a 4-channel Matrix using cv:merge().
				cv::Mat multiChannelMat = convertToMultiChannel(pnts4D);
				
				//convert output to 3D cartesian coordinates (x,y,z)
				cv::Mat pnts3D(1,undistortedCorners1.size(),CV_64FC3);
				cv::convertPointsFromHomogeneous(multiChannelMat,pnts3D);
					
		
				//cout << "pnts4D = \n" << pnts4D << "\n";cout << "pnts3D = \n" << pnts3D << "\n";

				// Display (x,y,z) coordinates on the image(s)
				// Let's just try for 1/3 points
				for(unsigned i=0, len = observedCorners1.size(); i < len ; i+=len/3)
				{
					ss << "( " << pnts3D.at<float>(i,0) <<" , " << pnts3D.at<float>(i,1) << " , " << pnts3D.at<float>(i,2) <<" )" ;
					
					//cout << "(x,y,z) =" << pnts3D.rowRange(i,i+1) << "\n";
					cv::circle(img1, undistortedCorners1[i], 3, GREEN, -1, CV_AA);
					cv::putText(img1,ss.str(),undistortedCorners1[i], cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, RED);
					//cv::putText(img2,ss.str(),observedCorners2[i], cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, RED);
					

					ss.str(""); //clear the stringstream buffer
				}
				

			}
	
			else
			{
				cout << "No corresponding features found!" << "\n";
			}

			imshow("Triangle - Left camera view", img1);
			imshow("Triangle - Right camera view", img2);

			char key = (char)waitKey(wait_key_delay);
		
			if(key == 'q' || key == 27)
			{
				break;
			}
	
		}

		vid1.release(); vid2.release();

	}

	else
	{
		cout << "Can't open video device input Can't triangulate,...";
	}

	
	
} 






//======================= Disparity Section ============================================



int checkParameters(const std::string& img1_filename, const std::string& img2_filename,const std::string& intrinsic_filename,const std::string& extrinsic_filename,const std::string& point_cloud_filename)
{
	if ( numberOfDisparitiesVal < 1 || numberOfDisparitiesVal % 16 != 0 )
    {
        printf("Function parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
        return -1;
    }

   
    if (SADWindowSizeVal < 1 || SADWindowSizeVal % 2 != 1)
    {
        printf("The block size (--blocksize=<...>) must be a positive odd number\n");
        return -1;
    }

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


//enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
// from $OPENCV_DIR/samples/cpp/stereo_match.cpp
int findDisparityAndDepth(const std::string& img1_filename, const std::string& img2_filename,
					 const std::string& intrinsic_filename,const std::string& extrinsic_filename,
					const std::string& disparity_file,const std::string& pointcloud_filename,float scale=1.0f)
{
	

	Ptr<StereoBM> bm = StereoBM::create(16,9);
	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
	
	
	if( checkParameters(img1_filename,img2_filename,intrinsic_filename ,extrinsic_filename, pointcloud_filename) == -1)
	{return -1;}


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
	int sgbmWinSize = SADWindowSizeVal;
    sgbm->setBlockSize(SADWindowSizeVal);

    int cn = img1.channels();
    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);

    sgbm->setMinDisparity(minDisparityVal);
    sgbm->setNumDisparities(numberOfDisparitiesVal);
    sgbm->setUniquenessRatio(uniquenessRatioVal);
    sgbm->setSpeckleWindowSize(speckleWindowSizeVal);
    sgbm->setSpeckleRange(setSpeckleRangeVal);
    sgbm->setDisp12MaxDiff(1);
    if(algVal==STEREO_HH)
        sgbm->setMode(StereoSGBM::MODE_HH);
    else if(algVal==STEREO_SGBM)
        sgbm->setMode(StereoSGBM::MODE_SGBM);
    else if(algVal==STEREO_3WAY)
        sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);


	 cv::Mat disp, disp8;

   // calculate time required to compute disparity using one of the BM algorithms
	int64 t = getTickCount();
   
    	sgbm->compute(img1, img2, disp);
    t = getTickCount() - t;
    fprintf(stderr,"Time elapsed: %fms\n", t*1000/getTickFrequency());
	
	disp.convertTo(disp8, CV_8U);

	//display 
	namedWindow("left", 1);
    imshow("left", img1);
    namedWindow("right", 1);
    imshow("right", img2);
    namedWindow("disparity", 0);
    imshow("disparity", disp8);
    fprintf(stderr,"Please re-tune parameters if needed and/or press a key to continue ...\n");
    char ch =waitKey(0);
	if(ch == 'q')
	{
		return -1;
	}
    printf("\n");


	//if(!disparity_file.empty()) { imwrite(disparity_file, disp8); }

	
}



/*
enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
// variables needed by trackbar
int algVal                 = STEREO_SGBM;
int SADWindowSizeVal       = algVal == STEREO_SGBM ? 3 : 9;
int preFilterCapVal        = 63;
int minDisparityVal        = 0 ; 
int numberOfDisparitiesVal = 16;
int uniquenessRatioVal     = 10; // 5-15%
int speckleWindowSizeVal   = 100; // 50-200
int setSpeckleRangeVal     = 32;  // 1 or 2
*/

void createStereoTrackBars(const char* trackBarWindowName) 
{

	createTrackbar("alg",trackBarWindowName,&algVal,4,0);
	createTrackbar("SADWindowSize(odd)",trackBarWindowName,&SADWindowSizeVal,15,0) ;
	createTrackbar("preFilterCap",trackBarWindowName,&preFilterCapVal,100,0) ;
	createTrackbar("minDisparity",trackBarWindowName,&minDisparityVal,20,0) ;
	createTrackbar("numberOfDisparities(%16=0)",trackBarWindowName,&numberOfDisparitiesVal,160,0) ;
	createTrackbar("uniquenessRatio(5-15)",trackBarWindowName,&uniquenessRatioVal,16,0) ;
	createTrackbar("speckleWindowSize(50-200)",trackBarWindowName,&speckleWindowSizeVal,200,0) ;
	createTrackbar("setSpeckleRange",trackBarWindowName,&setSpeckleRangeVal,5,0) ;

}



void doDisparity(const char* intrinsics_file,const char* extrinsics_file)
{
	std::string pointcloud_file = "point_cloud.txt"; 
	std::string disparity_file = "disparity_img.jpg";
	std::string left_img_file = "left_img.jpg";
	std::string right_img_file = "right_img.jpg";

	// Create a trackbar for parameter adjustment	
	const char* trackBarWindowName = "Stereo_TrackBar";
	cv::namedWindow(trackBarWindowName/*,WINDOW_AUTOSIZE*/);
	createStereoTrackBars(trackBarWindowName);
	
	// temporary buffers (used inside loop)
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
				
				if(findDisparityAndDepth(left_img_file, right_img_file,intrinsics_file,extrinsics_file,
									disparity_file,pointcloud_file) == -1 )
				{
					break;		
				}
		
			}
			
		}
	}

}


//========================= Top Level Function ===========================



int main( int argc, const char** argv )
{
	// Files containing intrinsics ( Optional input to cv::stereoCalibrate )
	char* internal_calibFile1 = "/home/malhar/FYP_opencv/saved_camera_params/logitech_microsoft/logitech_camera_params.xml";
	char* internal_calibFile2 = "/home/malhar/FYP_opencv/saved_camera_params/logitech_microsoft/microsoft_lifecam.xml";

	// We run calibration only if we are able to find corners in the input image of chessboard

	int num_images = 50; //number of images to be used for calibration

	int do_stereoCalibration = 0; // by default no stereo calibration
	int do_rectification = 0 ;    // by default no stereo rectifcation

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
	
	
	// files containing intrinsics and extrinsics of both cameras
	char* intrinsics_file = "Intrinsics.xml"; // These will be improvised intrinsics, byproduct of stereoCalibrate
	char* extrinsics_file = "extrinsics.xml"; // Main output of stereoCalibrate & stereoRectify

	if(do_stereoCalibration)
	{
		bool foundImagePoints =  load_image_points(num_images); 

		if(foundImagePoints)
		{
			cout << " Image points have been succesfully detected." << "\n";
			RunAndSaveStereoCalibration(internal_calibFile1,internal_calibFile2,intrinsics_file,extrinsics_file);
		}
	}
	
	
	if(do_rectification)
	{
		rectifyImages(intrinsics_file,extrinsics_file,false);
	}

	// Try either of the two - maybe do triangulation just to see how the 3D points are
	doTriangulate(intrinsics_file, extrinsics_file);

	//doDisparity(intrinsics_file,extrinsics_file);

	cout << "Finished." << "\n";

	return 0;

}

