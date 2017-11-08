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

const float calibSquareSize = 0.02578f; //metres
const Size chessBoardDimension(6,9); //width = 6 , height = 9 
int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
const Scalar RED(0,0,255), GREEN(0,255,0); 



bool myFunc(vector< vector<Point2f> >& imagePoints);


// this calculates 3D board positions - hardcoded stuff
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


// The main camera calibration function
// We run this after obtaining corners from several images
bool myRunCalibration(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                       vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs,
					vector<Mat>& tvecs,vector<float>& reprojErrs, double& totalAvgErr)
{
	cout << "Inside run calibration" << "\n";
	double rms;

	cameraMatrix = Mat::eye(3, 3, CV_64F);
	// According to doc , this can be 4, 5, 8 or 12 x 1 matrix/col vector
	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	calcKnownBoardCornerPositions(chessBoardDimension,calibSquareSize,objectPoints[0]);
	
	// imp step , we create same 3D points for each set of 2D corners in each image.
	objectPoints.resize(imagePoints.size(),objectPoints[0]);
	cout << "Inside run calibration ... going to calibrate camera" << "\n";

	cout << "image points size = " << imagePoints.size() << "\n";
	cout << "object points size = " << objectPoints.size() << "\n";

	rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
	cout << "Finshed camera calibrate !!" << "\n";
	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
	
	return ok;
	
}

void  mySaveCameraParams(Size imageSize, Mat cameraMatrix, Mat distCoeffs, vector<Mat> rvecs, vector<Mat> tvecs, vector<float> reprojErrs, vector<vector<Point2f> > imagePoints,double totalAvgErr)
{
	stringstream ss; 

	ofstream outfile("calibratedParams.txt");
	if(!outfile.is_open())
	{
		cout << "File cannot be opened ..exiting ..";
		exit(1);	
	}

	ss << "size of camera matrix = [" << cameraMatrix.rows << " x " << cameraMatrix.cols << "]" << std::endl ;
	
	ss << "Matrix elements = " << cameraMatrix;

	outfile << ss.str();
	outfile.close();
}

void myRunCalibrationAndSave(Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints)
{
	cout << "Inside run and save calibration" << "\n";
	/*
		These are the rotation and tangential matrices
		They convert the chessBoard from model coordinate space 
		to the 3D space of the camera (after this point all the 
		camera matrix and similarity of triangle stuff for pinhole camera
		makes sense ) 
	*/
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = myRunCalibration(imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr);

    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr << endl;

	/*
    if (ok)
    {
		 mySaveCameraParams(imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,totalAvgErr);
	}
	*/

}


int main( int argc, const char** argv )
{
	 bool calibrated =  false;
	vector< vector<Point2f> > imagePoints;

	calibrated = myFunc(imagePoints);

	if( calibrated)
	{
		
		Mat cameraMatrix,distCoeffs;

		myRunCalibrationAndSave(chessBoardDimension,cameraMatrix,distCoeffs,imagePoints);
	}

	else 
	{
		cout << "did not calibrate ..." << endl;
	}

	return 0;
}



bool myFunc(vector< vector<Point2f> >& imagePoints)
{

	
	unsigned totalImages = 0 ;

	VideoCapture vid(1);
	if(!vid.isOpened())
	{
		return -1;
	}

	Mat frame;
	Mat dupFrame; // since we dont want to modify original frame
	
	bool found;

	string msg = "Found Image !";

	int baseLine = 0;
    Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
   
	for(;;)
	{
		if(!vid.read(frame))
		{
			break;
		}

		dupFrame = frame.clone();

		vector<Point2f> pointBuf;
		found = false;

		// probably uses Harris detector + subPixel corner refinement
		found = findChessboardCorners(frame, chessBoardDimension, pointBuf, chessBoardFlags);

		if(found)
		{
            cvtColor(frame, dupFrame, COLOR_BGR2GRAY);
	
			// iterative corner refinement ... pointBuf is an InputOutputArray
            cornerSubPix( dupFrame, pointBuf, Size(11,11),
                Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));

			imagePoints.push_back(pointBuf);

			// Draw the corners for current frame
            drawChessboardCorners(dupFrame, chessBoardDimension, Mat(pointBuf), found);
			
		}

		imshow("Found Corners ?", dupFrame);
		char key = waitKey(100);
	
		
		switch(key)
		{
			case ' ':
				if(found)
				{
					/*
					Point textOrigin(dupFrame.cols - 2*textSize.width - 10, dupFrame.rows - 2*baseLine - 10);
					putText(dupFrame, msg, textOrigin, 1, 1,GREEN );
					*/
					totalImages++;
					
				}
				std::cout << "Current total = " << totalImages << "\n";
				if(totalImages > 10)
				{	
					vid.release();

					return true;
				}
				break ; 
				
				
			case 27:
				vid.release();
				return false;	
				break ;	
				
		}
		
		
	
	}

	

	return 0;

}
