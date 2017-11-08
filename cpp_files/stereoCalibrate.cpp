#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;



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
const Scalar RED(0,0,255), GREEN(0,255,0); 
vector< vector< Point2f > > left_img_points, right_img_points; //stores all corners
Size imageSize;
int num_images = 50 ;
int wait_key_delay = 1000;

// Function prototypes
bool load_image_points(int num_imgs) ;



#include "/home/malhar/opencv-3.3.0/apps/interactive-calibration/rotationConverters.hpp"

#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#define CALIB_PI 3.14159265358979323846
#define CALIB_PI_2 1.57079632679489661923

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
// It is then replicated for all images/views.
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


// This function only loads images taken from stereo cameras.
// In order to use it, images from both cameras must be present.
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
			}


			Mat gray1,gray2;
			cvtColor(img1, gray1, CV_BGR2GRAY);
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
		
	}


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

// save camera matrix and distortion coeffs.
void saveIntrinsicParameters(Mat cameraMatrix1,Mat cameraMatrix2,Mat distCoeffs1, Mat distCoeffs2)
{
    FileStorage fs("Intrinsics.xml", FileStorage::WRITE);
	
	if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix1 << "D1" << distCoeffs1 <<
            "M2" << cameraMatrix2 << "D2" << distCoeffs2;
        fs.release();
    }
    else
	{
        cout << "Error: can not save the intrinsic parameters\n";
	}

}

// save camera matrix and distortion coeffs.
void saveExtrinsicParameters(Mat R,Mat T,Mat R1, Mat R2,Mat P1, Mat P2, Mat Q)
{
	FileStorage fs("extrinsics.yml", FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
	{
        cout << "Error: can not save the extrinsic parameters\n";
	}

}



int RunAndSaveStereoCalibration(char* leftCalibFile, char* rightCalibFile)
{
	
	
	// Create object points, can use either left or right imagepoint list to resize
	vector<vector<Point3f> > objectPoints(1);
	calcKnownBoardCornerPositions(chessBoardDimension,calibSquareSize,objectPoints[0]);
	objectPoints.resize(left_img_points.size(),objectPoints[0]);
	

	// Read some parameters from the input file
	FileStorage fs1;
	fs1.open(leftCalibFile, FileStorage::READ);	

	if(!fs1.isOpened())
	{
		cout << "Please supply LEFT camera internal calibration parameters! "<< "\n";
		return -1;
	}

	Mat cameraMatrix1, distCoeffs1;
	
	fs1["camera_matrix"] >> cameraMatrix1 ;
	fs1["distortion_coefficients"] >> distCoeffs1;

	cout << "\n" << "camera Matrix 1 = "<< "\n" << cameraMatrix1 << "\n";
	cout << "\n" << "distortion coefficeints 1 = "<< "\n" << distCoeffs1 << "\n";

	fs1.release();  

	FileStorage fs2;
	fs2.open(rightCalibFile, FileStorage::READ);	

	if(!fs2.isOpened())
	{
		cout << "Please supply RIGHT camera internal calibration parameters! "<< "\n";
		return -1;
	}

	Mat cameraMatrix2, distCoeffs2;
	
	fs2["camera_matrix"] >> cameraMatrix2 ;
	fs2["distortion_coefficients"] >> distCoeffs2;

	cout << "\n" << "camera Matrix 1 = "<< "\n" << cameraMatrix2 << "\n";
	cout << "\n" << "distortion coefficients 1 = "<< "\n" << distCoeffs2 << "\n";

	fs2.release();  



	// rotation and translation between left and right camera.
	// Can think of the rotation vector as rotation required to 
	// convert camera1 orientation into camera 2.
	cv::Mat R,T,E,F;
	
	double rms = cv::stereoCalibrate(objectPoints,left_img_points,right_img_points,
	cameraMatrix1,distCoeffs1,cameraMatrix2,distCoeffs2,imageSize,
	R, T, E, F,
	CV_CALIB_USE_INTRINSIC_GUESS /*CALIB_FIX_INTRINSIC*/,
	cv::TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );

	cout << "stereo calibration re-projection error = " << rms << "\n" ;
	cout << "Rotation vector, size = " << R.size() << "\n" << R << "\n" ;
	// convert rotation vector to euler angles ..?
	Mat rotVector ;

	// NOTE - There is a difference between Rodrigues and Euler angles.
	myRot2Euler(R,rotVector); 
	cout << "Euler angles with size = " << rotVector.size() << "\n" << rotVector << "\n" ;
	cout << "Translation vector" << "\n" << T << "\n" ; 

	// save intrinsic parameters - why before stereoRectify() ?
	saveIntrinsicParameters(cameraMatrix1,cameraMatrix2,distCoeffs1,distCoeffs2);

	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

    stereoRectify(cameraMatrix1, distCoeffs1,
                 cameraMatrix2, distCoeffs2,
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
	
	// save extrinsic parameters
	saveExtrinsicParameters(R,T,R1,R2,P1,P2,Q);
	
} 



int main( int argc, const char** argv )
{

	
	char* calibFile1 = "/home/malhar/opencv_work/saved_camera_params/microsoft_lifecam.xml";
	char* calibFile2 = "/home/malhar/opencv_work/saved_camera_params/logitech_camera_params.xml";

	bool foundImagePoints =  load_image_points(num_images); 

	if(foundImagePoints)
	{
		cout << " Image points have been succesfully detected." << "\n";
		RunAndSaveStereoCalibration(calibFile1,calibFile2);
	}
	

	/*
	Mat matr(3,3,CV_64FC1, Scalar(-1));

	matr.at<double>(0,0) =  0.9995084336637333f ; 
	matr.at<double>(0,1) =   -0.001613575022304045f ; 
	matr.at<double>(0,2) =  -0.03130954184777966f ; 
	matr.at<double>(1,0) =  -0.00559784559813541f ; 
	matr.at<double>(1,1) =  0.9734409084064423f ; 
	matr.at<double>(1,2) =  -0.2288699673733975f ; 
	matr.at<double>(2,0) =  0.03084728772080141f ; 
	matr.at<double>(2,1) =  0.2289327285830666f ; 
	matr.at<double>(2,2) =   0.9729533650816888f ; 

	cout << " rot matric with size =" << matr.size() << "\n" << matr << "\n" ; 	
	Mat rotVector; 
	myRot2Euler(matr,rotVector);
	//Rodrigues(rotVector,matr);
	
	cout << " rot matric with size =" << matr.size() << "\n" << matr << "\n" ; 	
	cout << "Euler angles = " << "\n" << rotVector.size() << "\n" << rotVector << "\n" ;
	
	*/


	return 0;

}

