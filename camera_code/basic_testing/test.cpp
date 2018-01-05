#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <sstream>
using namespace std;
using namespace cv;


void captureImages()
{

	int counter = 1;
	
	//std::string vid_left = "video/drone_left.avi";
	//std::string vid_right = "video.drone_right.avi";

	cv::VideoCapture vid1(1);
	cv::VideoCapture vid2(2);

		
	if(vid1.isOpened() && vid2.isOpened())
	{

		cv::Mat left_img, right_img;
		
		std::cerr << "Please press 'c'once to capture both images ..." << "\n";
		while(1)
		{
			if(vid1.read(left_img) && vid2.read(right_img))
			{
				namedWindow("L",CV_WINDOW_AUTOSIZE);
				namedWindow("R",CV_WINDOW_AUTOSIZE);

				imshow("L",left_img);
				imshow("R",right_img);
			
				
				char ch = (char)waitKey(100);
				if(ch == 'c')
				{
					std::stringstream ss;
					ss << "left_img" << counter << ".jpg" ;
					imwrite(ss.str(),left_img);	
					ss.str("");
	
					ss << "right_img"<< counter << ".jpg";
					imwrite(ss.str(),right_img);
					break;
				}
	
				else if(ch == 'q')
				{
					break;
				}
			} 	
		}	
		
	}

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


void findChannels(cv::Mat mat)
{
	cout << "Size of matrix = [" << mat.size().height << " x " << mat.size().width <<"]"  << "\n";
    std::cout << "Channels: " << mat.channels() << std::endl;
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




// Note - Returns true if input file is not having a reprojection entry field. ( In which case it will possibly be overwritten
// by whoever is using this function.)
bool betterThanPreviousReprojectionError(const char* file, double currentReprojectionError)
{
	bool betterThanPrevious = false;

	FileStorage fs;
	fs.open(file,FileStorage::READ);

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



int main(int argc, char* argv[])
{
	/*
	Mat M = cv::Mat::eye(2,5,CV_8UC1);
    cout << "M = "<< endl << " "  << M << endl << endl;
	findChannels(M);
	
	// Convert a single channel array into multi-channel array
	Mat R1 = M.row(0);
	Mat R2 = M.row(1);
	Mat res4;

	cout << "R1 = " << R1 << "\n";
	cout << "R2 = " << R2 << "\n";

	std::vector<cv::Mat> channels;
	channels.push_back(R1);
	channels.push_back(R2);

	cv::merge(channels,res4);
	
	cout << "res4 =" << res4 << "\n";
findChannels(res4);

	 findChannels(convertToMultiChannel(M));
	

	std::vector<Point2f> vec;
	vec.push_back(Point2f(1.0,2.0));
	vec.push_back(Point2f(3.0,4.0));
	vec.push_back(Point2f(5.0,6.0));
	
	cv::Mat res = convertPoint2fTo2ChannelMatrix(vec);
	findChannels(res);

	cout << "res = " << res << "\n";
	 
	

	
	cv::Mat K1,K2,R1,R2,R,T;
	K1 = cv::Mat(3,2,CV_8UC1);
	K1.at<uchar>(1,1) = 12;
	cout << K1 << "\n";
	double currentReprojectionError = 0;
	FileStorage fs1;
	
		
	if(betterThanPreviousReprojectionError("Intrinsics.xml",currentReprojectionError))
	{
		fs1.open("Intrinsics.xml", FileStorage::WRITE);	
		if(fs1.isOpened())
		{
			fs1 << "K1" << K1 ;

			fs1 << "reprojError" << currentReprojectionError ; 
			fs1.release();	
		}
	}	
	

	else 
	{
		cout << "Not going to overwrite! "<< "\n";
	}
	 
	*/
	captureImages();

	/*
	FileStorage fs2;
	fs2.open("extrinsics.xml", FileStorage::READ);	

	if(!fs2.isOpened())
	{
		cout << "Please supply RIGHT camera internal calibration parameters! "<< "\n";
		return -1;
	}

	fs2["R"] >> R ;
	fs2["R1"] >> R1 ;
	fs2["R2"] >> R2;
	fs2["T"] >> T;

	fs2.release();  

	cout << "K1= " << K1 << "\n";

	cout << "K1 ro1= " << K1.colRange(0,1) << "\n";
	cout << "K1 ro1= " << K1.colRange(0,2) << "\n";

    cv::Mat Proj =  computeProjectionMatrix(K1,R,T);
	*/
	//cout << "Proj = " << Proj << "\n";
	
	
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
