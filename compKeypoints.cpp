//#include "droneVisionNew.h"
#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <stdio.h>
using namespace cv;
void calcDescriptors(Mat Image)
{
	//prinf("calclulating Descriptors");
	const Mat input = Image; //Load as grayscale

    SiftFeatureDetector detector;
    std::vector<KeyPoint> keypoints;
    detector.detect(input, keypoints);

    // Add results to image and save.
	Mat output;
    drawKeypoints(Image, keypoints, output);
	imshow( "Keypoints", output );
	cvWaitKey(0);
	
	
	cvDestroyWindow( "Keypoints" );
}