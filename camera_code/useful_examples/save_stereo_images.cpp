#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>

#include <cmath>

#include <opencv2/calib3d.hpp>
using namespace std;
using namespace cv;
int main()
{

	cv::VideoCapture vid1(1);
	cv::VideoCapture vid2(2);

		
	if(vid1.isOpened() && vid2.isOpened())
	{

		cv::Mat left_img, right_img;

		while(1)
		{
			if(vid1.read(left_img) && vid2.read(right_img))
			{
				namedWindow("left",CV_WINDOW_AUTOSIZE);
				namedWindow("right",CV_WINDOW_AUTOSIZE);

				imshow("L",left_img);
				imshow("R",right_img);
			
				char ch = (char)waitKey(100);
				if(ch == 'T')
				{
					imwrite("left_img.jpg",left_img);	
					imwrite("right_img.jpg",right_img);
				}
	
				if(ch == 'q')
				{
					break;
				}
			} 	
		}	
		
	}

}
