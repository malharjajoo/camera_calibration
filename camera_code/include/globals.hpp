#ifndef GLOBALS
#define GLOBALS

#include <opencv2/core.hpp>


#define CALIB_PI 3.14159265358979323846
#define CALIB_PI_2 1.57079632679489661923
#define FONT_SIZE 0.4

// A few globals for ease - place in common globals file.
const float calibSquareSize = 0.02578f; //in metres
const cv::Size chessBoardDimension(6,9); //width = 6 , height = 9 
//int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
const cv::Scalar RED(0,0,255), GREEN(0,255,0), BLUE(255,0,0);
const int esc_ascii = 27;

// For HSV thresholding
extern int h_min ; 
extern int h_max ; 
extern int s_min ;
extern int s_max;
extern int v_min ; 
extern int v_max;  



#endif
