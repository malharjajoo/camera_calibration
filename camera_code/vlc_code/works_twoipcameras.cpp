#include <stdio.h>
 #include <stdlib.h>
 #include <vlc/vlc.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>
#include <mutex>
#include <iostream>


using namespace cv;
using namespace std; 


std::mutex imageMutex;
std::mutex imageMutex_2;

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



void *lock_2(void *data, void**p_pixels) 
{
	struct ctx *ctx = (struct ctx*)data;

	// Locking
    imageMutex_2.lock();//WaitForSingleObject(ctx->imageMutex, INFINITE); 

// pixel will be stored on image pixel space
	*p_pixels = ctx->pixels; 
	return NULL; 
}

void display_2(void *data, void *id)
{
	(void) data; assert(id == NULL);
}

void myunlock_2(void *data, void *id, void *const *p_pixels)
{ 
	// get back data structure 
	struct ctx *ctx = (struct ctx*)data; /* VLC just rendered the video, but we can also render stuff */ 

	// show rendered image 
	uchar *pixels = (uchar*)*p_pixels;
 	//imshow("myTestImage", *ctx->image); waitKey(2); // wait 100ms for Esc key 
	imageMutex_2.unlock(); //ReleaseMutex(ctx->imageMutex); 

} 


//=====================================================


// func_both
void func_both()
{
	// VLC pointers
	libvlc_instance_t *vlcInstance;
	libvlc_instance_t *vlcInstance_2 ;

	libvlc_media_player_t *mp;
	libvlc_media_player_t *mp_2;

	libvlc_media_t *media;
	libvlc_media_t *media_2;

	const char * const vlc_args[] = { 
		"-I", "dummy", // Don't use any interface 
		"--ignore-config", // Don't use VLC's config 
		"--extraintf=logger", // Log anything 
		"--verbose=2", // Be much more verbose then normal for debugging purpose 
  };

	vlcInstance = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);
	vlcInstance_2 = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);

	// Read a distant video stream //
	media = libvlc_media_new_location(vlcInstance, "rtsp://192.168.1.101:554/onvif1"); 
	media_2 = libvlc_media_new_location(vlcInstance, "rtsp://192.168.1.102:554/onvif1"); 

	mp = libvlc_media_player_new_from_media(media); 
	mp_2 = libvlc_media_player_new_from_media(media_2); 

	libvlc_media_release(media); 
	libvlc_media_release(media_2); 

	
	struct ctx* context = ( struct ctx* )malloc( sizeof( *context ) ); 
	struct ctx* context_2 = ( struct ctx* )malloc( sizeof( *context_2 ) ); 

	
	context->image = new Mat(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3); 
	context->pixels = (unsigned char *)context->image->data;	

	context_2->image = new Mat(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3); 
	context_2->pixels = (unsigned char *)context_2->image->data;	

	
	// Key part of the code.
	libvlc_video_set_callbacks(mp, lock, myunlock, display, context);
	libvlc_video_set_callbacks(mp_2, lock_2, myunlock_2, display_2, context_2);

	libvlc_video_set_format(mp, "RV24", VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_WIDTH * 24 / 8); // pitch = width * BitsPerPixel / 8
	libvlc_video_set_format(mp_2, "RV24", VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_WIDTH * 24 / 8);


  int ii = 0; int key = 0; 
	while(key != 27) { ii++; 
			if(ii > 5) { 

				libvlc_media_player_play(mp);
				libvlc_media_player_play(mp_2);

				float fps = libvlc_media_player_get_fps(mp); printf("fps:%f\r\n",fps); 
				imshow("Camera_1", *context->image);
				imshow("Camera_2", *context_2->image);
 
			} 
	
			
			key = waitKey(10);
	}

   libvlc_media_player_stop (mp);
	 libvlc_media_player_stop (mp_2);

   libvlc_media_player_release (mp);
	 libvlc_media_player_release (mp_2);

   libvlc_release (vlcInstance);	
	 libvlc_release (vlcInstance_2);

}



// compile and run using 
// g++ -std=c++11 <filename>.cpp -o <exe> $(pkg-config --cflags libvlc --libs libvlc --cflags opencv --libs opencv) -lpthread; ./<exe>
int main() 
{ 

	cv::namedWindow("Camera_1", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Camera_2", CV_WINDOW_AUTOSIZE);

	func_both();
 
	return 0;

 }

