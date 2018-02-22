#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

#include <stdio.h>
 #include <stdlib.h>
 #include <vlc/vlc.h>
#include <opencv2/core.hpp>
#include <unistd.h>
#include <mutex>
#include <iostream>



using namespace cv;
using namespace std; 


std::mutex imageMutex;

struct ctx 
{ 
 	Mat* image;
  uchar* pixels;
};



 // define output video resolution 
#define VIDEO_WIDTH 640 
#define VIDEO_HEIGHT 480 
void *lock(void *data, void**p_pixels) 
{
	struct ctx *ctx = (struct ctx*)data;

	// Locking
    imageMutex.lock();//WaitForSingleObject(ctx->imageMutex, INFINITE); 

// pixel will be stored on image pixel space
	*p_pixels = ctx->pixels; 
	return NULL; 
}

void display(void *data, void *id)
{
	(void) data; assert(id == NULL);
}

void myunlock(void *data, void *id, void *const *p_pixels)
{ 
	// get back data structure 
	struct ctx *ctx = (struct ctx*)data; /* VLC just rendered the video, but we can also render stuff */ 

	// show rendered image 
	uchar *pixels = (uchar*)*p_pixels;
 	//imshow("myTestImage", *ctx->image); waitKey(2); // wait 100ms for Esc key 
	imageMutex.unlock(); //ReleaseMutex(ctx->imageMutex); 

} 


//==========================================


/* This function publishes IP camera image over ROS topics
   It uitlizes image_tranport package from ROS.
	 use rostopic list in a new terminal to see published topics.
*/



int main(int argc, char** argv)
{

  // Optionaly use this if using cv::imshow later below
	//cv::namedWindow("Camera_1", CV_WINDOW_AUTOSIZE);


  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(5);


//====================================

	// VLC pointers
	libvlc_instance_t *vlcInstance;
	libvlc_media_player_t *mp;
	libvlc_media_t *media;

	const char * const vlc_args[] = { 
		"-I", "dummy", // Don't use any interface 
		"--ignore-config", // Don't use VLC's config 
		"--extraintf=logger", // Log anything 
		"--verbose=2", // Be much more verbose then normal for debugging purpose 
  };

	vlcInstance = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);

	// Read a distant video stream //
	// NOTE: If connected via ethernet, to find IP address of camera
	// 1) run "ifconfig". Check ethernet Broadcast (BCast) address.
  // 2) ping -b BROADCAST_ADDRESS
	// 3) arp -a
	// 4) ping some of the results from arp. 
	// (^ Maybe MAC address can be used to identify ip camera in step 4) )
 
	std::string IP_address="rtsp://10.42.0.16:554/onvif1";
	media = libvlc_media_new_location(vlcInstance, IP_address.c_str()); 
 

	mp = libvlc_media_player_new_from_media(media); 
	libvlc_media_release(media); 

	
	struct ctx* context = ( struct ctx* )malloc( sizeof( *context ) ); 

	//context->imageMutex = CreateMutex(NULL, FALSE, NULL); 
	context->image = new Mat(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3); 
	context->pixels = (unsigned char *)context->image->data;	

	// show blank image
	//imshow("test", *context->image);


	libvlc_video_set_callbacks(mp, lock, myunlock, display, context);
	libvlc_video_set_format(mp, "RV24", VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_WIDTH * 24 / 8); // pitch = width * BitsPerPixel / 8


//=============================


  int ii = 0;
	// int key = 0; use for waitKey if using cv::imshow() below.
	while(nh.ok()) { ii++; 
			if(ii > 5) { 
	
				libvlc_media_player_play(mp);
				float fps = libvlc_media_player_get_fps(mp); printf("fps:%f\r\n",fps); 


				// Check if grabbed frame is actually full with some content
				if(!(*context->image).empty()) 
				{
				  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *context->image).toImageMsg();
				  pub.publish(msg);
				  cv::waitKey(1);
				}

			} 
	
			
			//key = waitKey(10);

			 ros::spinOnce();
    	loop_rate.sleep();
	}


	 libvlc_media_player_stop (mp);

   libvlc_media_player_release (mp);

   libvlc_release (vlcInstance);


}



