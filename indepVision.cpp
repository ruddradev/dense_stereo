#include <cvaux.h>
#include <highgui.h>
#include "compKeypoints.h"
using namespace cv;
void getImage()
{
Mat	img=imread("C:/mimg/1.jpg");
namedWindow( "Example1", CV_WINDOW_AUTOSIZE );
imshow( "Example1", img );
calcDescriptors(img);
cvWaitKey(0);
//releaseImage( &Image );
//cvDestroyWindow( "Example1" );
}