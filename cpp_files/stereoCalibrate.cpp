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

#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#define CALIB_PI 3.14159265358979323846
#define CALIB_PI_2 1.57079632679489661923
#define FONT_SIZE 0.5



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
int num_images = 20;
int wait_key_delay = 1000;
std::vector<Mat> goodImageList; // list of "pair" of images where corners were detcted

// Function prototypes
bool load_image_points(int num_imgs) ;




//==================== Helper functions ===================


// Computes P = K * [R|t]
// K : cameraMatrix , 3x3
// R : rotationMatrix , 3x3
// t : translation matrix , 3x1
cv::Mat computeProjectionMatrix(cv::Mat K,cv::Mat R,cv::Mat t)
{
	cv::Mat RTMat(3, 4, CV_64F);
	cv::hconcat(R,t,RTMat);

	return K*RTMat;
}


void printMatrixInfo(cv::Mat M)
{
	cout << "rows = " << M.rows << "\n";
	cout << "cols = " << M.cols << "\n";
	cout << "channels = " << M.channels() << "\n";
}


// cv::Mat.size() gives [ width x col ]
void findSize(cv::Mat mat)
{
	cout << "Size of matrix = [" << mat.size().height << " x " << mat.size().width <<"]"  << "\n";
    //std::cout << "Channels: " << mat.channels() << std::endl;
}


cv::Mat convertToMultiChannel(cv::Mat m)
{
	cv::Mat resMultiChannel;
	std::vector<cv::Mat> channels;	

	for(unsigned i = 0 ;i < m.size().height ; i++)
	{
		channels.push_back(m.row(i));
	}
	cv::merge(channels, resMultiChannel);
	return resMultiChannel;
}



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


//==========================================================================

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
			cvtColor(img2, gray2, CV_BGR2GRAY);
	
			// store in vector for display later.
			goodImageList.push_back(gray1.clone());
			goodImageList.push_back(gray2.clone());

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

// save camera matrix and distortion coeffs.
void saveIntrinsicParameters(Mat cameraMatrix1,Mat cameraMatrix2,Mat distCoeffs1, Mat distCoeffs2)
{
    FileStorage fs("Intrinsics.xml", FileStorage::WRITE);
	
	if( fs.isOpened() )
    {
        fs << "cameraMatrix1" << cameraMatrix1 << "distCoeffs1" << distCoeffs1 <<
            "cameraMatrix2" << cameraMatrix2 << "distCoeffs2" << distCoeffs2;
        fs.release();
    }
    else
	{
        cout << "Error: can not save the intrinsic parameters\n";
	}

}




// save relative camera pose, rectification transformation matrices...
void saveExtrinsicParameters(Mat R, Mat T,Mat R1, Mat R2,Mat P1, Mat P2, Mat Q)
{
	cv::Mat rotVector_R ;
	cv::Mat rotVector_R1 ;
	cv::Mat rotVector_R2 ;

	// NOTE - There is a difference between Rodrigues and Euler angles.
	myRot2Euler(R,rotVector_R); 	
	myRot2Euler(R1,rotVector_R1); 
	myRot2Euler(R2,rotVector_R2); 
	
	FileStorage fs("extrinsics.xml", FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "Euler_R" << rotVector_R << "T" << T << "R1" << R1 <<  "Euler_R1" << rotVector_R1 << "R2" << R2 << "Euler_R2" << rotVector_R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
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
	//cout << "Rotation vector, size = " << R.size() << "\n" << R << "\n" ;
	// convert rotation vector to euler angles ..?
	
	cout << "Translation vector" << "\n" << T << "\n" ; 

	// save intrinsic parameters - why after stereoCalibrate ? Because it improves them ?
	saveIntrinsicParameters(cameraMatrix1,cameraMatrix2,distCoeffs1,distCoeffs2);

	cv::Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];


	// Image rectification 
    cv::stereoRectify(cameraMatrix1, distCoeffs1,
                 cameraMatrix2, distCoeffs2,
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	
	// save extrinsic parameters
	saveExtrinsicParameters(R,T,R1,R2,P1,P2,Q);

	 // this is used later to display images side by side or one below the other.
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));



	//compute maps for cv::remap()
	 Mat rmap[2][2];

	// we extract first 3 coloumns of P1 and P2, since they are the "new" camera matrix
    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);


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

	// iterate over pair of images
    for( int i = 0, len = goodImageList.size()/2 ; i < len ; i++ )
    { 
        Mat img1 = goodImageList[i*2];
		Mat rect_img1, cimg1;

		if( !img1.empty() )
		{
			// the remap function works as follows - 
			// dest(x,y) = src()
		    remap(img1, rect_img1, rmap[0][0], rmap[0][1], INTER_LINEAR);
		    cvtColor(rect_img1, cimg1, COLOR_GRAY2BGR);
		    Mat canvasPart1 = !isVerticalStereo ? canvas(Rect(0, 0, w, h)) : canvas(Rect(0, 0, w, h));
		    resize(cimg1, canvasPart1, canvasPart1.size(), 0, 0, INTER_AREA);
	
		}
		 else{ cout << "Image 1 is empty for i = !" << i << "\n"; }     

		Mat img2 = goodImageList[i*2+1];
	
		if( !img2.empty() )
		{
			Mat rect_img2, cimg2;

		    remap(img2, rect_img2, rmap[1][0], rmap[1][1], INTER_LINEAR);
		    cvtColor(rect_img2, cimg2, COLOR_GRAY2BGR);
		    Mat canvasPart2 = !isVerticalStereo ? canvas(Rect(w, 0, w, h)) : canvas(Rect(0, h, w, h));
		
		    resize(cimg2, canvasPart2, canvasPart2.size(), 0, 0, INTER_AREA);
		}
	
		else{ cout << "Image 2 is empty ! for i =" << i <<  "\n"; }


		// draw horizontal or vertical lines on the image
        if( !isVerticalStereo )
            for( int j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( int j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);

        char c = (char)waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
			
            break;
    }

	cv::destroyAllWindows();

//======================== Triangulation Section ===========================


	cout << "Entered triangulation Section ..." << "\n";
	VideoCapture vid1(1);
	VideoCapture vid2(2);
	
	if(vid1.isOpened() && vid2.isOpened() )
	{
		for(;;)
		{
			Mat img1,img2;
			vector< Point2f > observedCorners1, observedCorners2, undistortedCorners1, undistortedCorners2;

			if(!vid1.read(img1) || !vid2.read(img2) )
			{	
				cout << "Can't read images !!" << "\n";
				break;
			}
	

			bool found1 = false, found2 = false;

			found1 = cv::findChessboardCorners(img1, chessBoardDimension, observedCorners1,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
			found2 = cv::findChessboardCorners(img2, chessBoardDimension, observedCorners2,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);


			if(found1 && found2) 
			{

				Mat gray1,gray2;
				cvtColor(img1, gray1, CV_BGR2GRAY);
				cvtColor(img2, gray2, CV_BGR2GRAY);

				// store in vector for display later.
				goodImageList.push_back(gray1.clone());
				goodImageList.push_back(gray2.clone());

				cv::cornerSubPix(gray1, observedCorners1, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				cv::drawChessboardCorners(img1, chessBoardDimension, observedCorners1, found1);

				cvtColor(img2, gray2, CV_BGR2GRAY);
				cv::cornerSubPix(gray2, observedCorners2, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				cv::drawChessboardCorners(img2, chessBoardDimension, observedCorners2, found2);

				//========== Undistort and triangulate ======
	
				//convertPoint2fTo2ChannelMatrix ?? Do I need to do this ?
				cv::undistortPoints(observedCorners1,undistortedCorners1,cameraMatrix1,distCoeffs1,R1,P1);
				cv::undistortPoints(observedCorners2,undistortedCorners2,cameraMatrix2,distCoeffs2,R2,P2);

				//cout << "After undistort" << "\n";
				//cout << "Observed corners 1 = \n" << observedCorners1 << "\n";
				//cout << "uUndistorted corners 1 = \n" << undistortedCorners1 << "\n";

				cv::Mat pnts4D(1,undistortedCorners2.size(),CV_64FC4);

				/*
				cv::Mat projMat1,projMat2 ;
				projMat1 = computeProjectionMatrix(cameraMatrix1,R,T);
				projMat2 = computeProjectionMatrix(cameraMatrix2,R,T);
				*/				
				
				cv::triangulatePoints(P1,P2,undistortedCorners1,undistortedCorners2,pnts4D);
			
				//cout << "The 4D points are = " << "\n" << pnts4D << "\n"; 

				// since output of triangulation is 4XN single channel array 
				// we convert it to a 4-channel Matrix using cv::Merge.
				cv::Mat multiChannelMat = convertToMultiChannel(pnts4D);
				
				//convert output to standard 3D (x,y,z) as we know
				cv::Mat pnts3D(1,undistortedCorners1.size(),CV_64FC3);
				cv::convertPointsFromHomogeneous(multiChannelMat,pnts3D);
					
		
				//cout << "pnts4D = \n" << pnts4D << "\n";
				//cout << "pnts3D = \n" << pnts3D << "\n";

				// put (x,y,z) coordinate onto the image(s)
				// Let's just try for a few corners

				std::stringstream ss;  // used during display
				for(unsigned i=0, len = observedCorners1.size(); i < len ; i+=30)
				{
					ss << "( " << pnts3D.at<float>(i,0) <<" , " << pnts3D.at<float>(i,1) << " , " << pnts3D.at<float>(i,2) <<" )" ;

					
					//cout << "(x,y,z) =" << pnts3D.rowRange(i,i+1) << "\n";

					cv::putText(img1,ss.str(),observedCorners1[i], cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, cv::Scalar(0,0,255));
	
					cv::putText(img2,ss.str(),observedCorners2[i], cv::FONT_HERSHEY_SIMPLEX, FONT_SIZE, cv::Scalar(0,0,255));

					ss.str(""); //clear the stringstream buffer
				}
				

			}
	
			else
			{
				//cout << "No corners found!" << "\n";
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
		cout << "Can't triangulate, Can't open video device input'...";
	}


} 



int main( int argc, const char** argv )
{

	
	char* calibFile1 = "/home/malhar/opencv_work/saved_camera_params/logitech_camera_params.xml";
	char* calibFile2 = "/home/malhar/opencv_work/saved_camera_params/microsoft_lifecam.xml";

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

