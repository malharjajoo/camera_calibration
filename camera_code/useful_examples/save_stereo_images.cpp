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

#define SAVE_VIDEO 0

cv::Mat reduceBrightness(cv::Mat img)
{
	cv::Mat img2;
	img.convertTo(img2, -1, 1, -80);
	return img2;
}


int main()
{

/*
	std::string left_window_name = "Left camera";
	std::string right_window_name = "Right camera";
	
	cv::VideoCapture vid1(1);
	cv::VideoCapture vid2(2);

	#if SAVE_VIDEO == 1
		cv::VideoWriter vidWriter;
		std::string outputVideoName = "output.avi";
		

		Size S = Size((int) vid1.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
		                (int) vid1.get(CV_CAP_PROP_FRAME_HEIGHT));
		vidWriter.open(outputVideoName,CV_FOURCC('M','J','P','G'),10 , S, true);
	#endif

	if(vid1.isOpened() && vid2.isOpened())
	{

		cv::Mat left_img, right_img;

		namedWindow(left_window_name);
		namedWindow(right_window_name);

		cv::moveWindow(left_window_name, 20,20);
		cv::moveWindow(right_window_name, 700,20);

		while(1)
		{
			if(vid1.read(left_img) && vid2.read(right_img))
			{
		
				right_img = reduceBrightness(right_img);
				imshow(left_window_name, left_img);
				imshow(right_window_name, right_img);
			
				char ch = (char)waitKey(1);
				if(ch == 'T')
				{
					imwrite("left_img.jpg",left_img);	
					imwrite("right_img.jpg",right_img);
				}
	
				if(ch == 'q')
				{
					break;
				}
	
				#if SAVE_VIDEO == 1
					vidWriter << left_img;
				#endif

			} 	
		}	
		
	}
*/


	std::string IP_address = "10.42.0.16:554";
	std::string user_pwd = "admin:888888@";
	//std::string stream_address = "/onvif1";

	
	// "http://admin:888888@192.168.0.102:5000"
	//std::string vidInput = "rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov" ; // doesnt work if FFMPG=OFF


	std::vector<std::string> vp = {	
																//"rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov",
																//"video/single_drone_outdoor.mp4",
																	
																	//"rtsp://"+IP_address+"/onvif1" ,
																	//"rtsp://"+user_pwd+IP_address+"/onvif1",
		
																	"rtsp://"+IP_address+"/onvif1?tcp" ,
																	"rtsp://"+user_pwd+IP_address+"/onvif1?tcp"
																	
																};



	for(int i =0; i < vp.size(); ++i){

				std::string vidInput =  vp[i];

				std::cerr << "Trying to open rtsp url =" << vidInput << "\n";
				cv::VideoCapture vid1;
				vid1.set(CV_CAP_PROP_FOURCC, CV_FOURCC('H', '2', '6', '4'));
				vid1.open(vidInput); 
				//vid1.open(vidInput,cv::CAP_GSTREAMER); 


				if(vid1.isOpened())
				{
					std::cout << "Inside !!\n";
					cv::Mat left_img;

					while(1)
					{
						if(vid1.read(left_img) )
						{
							namedWindow("ip_cam",CV_WINDOW_AUTOSIZE);
				

							imshow("ip_cam",left_img);

							char ch = (char)waitKey(35);
			
	
							if(ch == 'q')
							{
								break;
							}
						} 	
					}	

				
		
				}
				else
				{
					std::cerr << "Can't open IP camera !!\n";
				}
	

	}




	/* 
	VideoCapture capture;
	Mat image; 

	// tried but didnt work 
		// http://USER:PWD@IPADDRESS:8088/mjpeg.cgi?user=USERNAME&password=PWD&channel=0&.mjpg
	
	
	
	std::string IP_address = "192.168.0.108";
	std::string PORT = "5000";

	
	if (capture.open("http://192.168.0.108:5000/video.cgi?.mjpeg")) {
		  cout << "Works!" << endl;
		  return -1;
	}




	else if (capture.open("http://192.168.0.108:5000/video?x.mjpeg")) {
		  cout << "Works!" << endl;
		  return -1;
	}
	

	else if (capture.open("http://192.168.0.108:5000/mjpeg/video.mjpeg")) {
		  cout << "Works!" << endl;
		  return -1;
	}

	else if (capture.open("http://192.168.0.108:5000/mjpeg.cgi?channel=0&.mjpeg")) {
		  cout << "Works!" << endl;
		  return -1;
	}

	else if (capture.open("http://192.168.0.108:5000/video.mjpeg")) {
		  cout << "Works!" << endl;
		  return -1;
	}

	
	else 
	{
		cout << "Nothing works ...  :( " << endl;
		return -1;
	}
	*/



}
