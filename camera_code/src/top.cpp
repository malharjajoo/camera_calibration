#include <iostream>

#include <cmath>




#include "StereoCamera.hpp"
#include "globals.hpp"



using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;



/*
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
				
				if(findDisparityAndDepth(left_img_file, right_img_file,intrinsics_file,extrinsics_file,
									disparity_file,pointcloud_file) == -1 )
				{
					break;		
				}
		
			}
			
		}
	}

}
*/

//========================= Top Level Function ===========================



int main( int argc, const char** argv )
{
	// Files containing intrinsics ( Optional input to cv::stereoCalibrate )
	char* internal_calibFile_left = "/home/malhar/FYP_opencv/saved_camera_params/logitech_microsoft/logitech_camera_params.xml";
	char* internal_calibFile_right = "/home/malhar/FYP_opencv/saved_camera_params/logitech_microsoft/microsoft_lifecam.xml";
	// files containing intrinsics and extrinsics of both cameras
	std::string intrinsics_file = "Intrinsics.xml"; // These will be improvised intrinsics, byproduct of stereoCalibrate
	std::string extrinsics_file = "Extrinsics.xml"; // Main output of stereoCalibrate & stereoRectify
	
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

