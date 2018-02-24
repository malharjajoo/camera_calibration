#ifndef DETECTOR
#define DETECTOR
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>


void createHSVTrackBar(const char* trackBarWindowName);

// C++ interface for all detectors
class Detector
{
	public:
		
		virtual cv::Rect detect(cv::Mat frame) =0;
		
		virtual void displayInfo()=0;
};


// Detects various other threats
class ThreatDetector:public Detector
{
	public:

		cv::Rect detect(cv::Mat frame) override;
		
		void displayInfo() override;

		
};


class DroneDetector:public Detector
{
	private:
		cv::Mat background_img;

	public:

		DroneDetector();

		cv::Rect detect(cv::Mat frame) override;

		void displayInfo() override;

		// helpers
		cv::Mat getBackgroundImage(cv::VideoCapture vid); //for background subtraction
	  
	  	void initializeBackground(cv::Mat bg_img);
};


#endif
