// This is the main DLL file.


#include <jni.h>
#include <vision_LoadImage.h>
#include <cvaux.h>
#include <highgui.h>
#include "compKeypoints.h"
#include "droneVisionNew.h"
#include "indepVision.h"
#include "keypoint.h"
#include "stereoCalibration.h"
#include "stereo1.h"
#include "SparseMatch.h"
;using namespace cv;
JNIEXPORT void JNICALL Java_vision_LoadImage_loadImage
  (JNIEnv *env, jclass cl, jstring fname)
{
	Mat img = imread( env->GetStringUTFChars(fname, JNI_FALSE) );
	StereoCalib("D:\\list.txt",6,9,2,1.0f,2);
	//convertQMatrix();
	//stereoCalib(9,6,5,IMLeft,IMRight, FMat,dMatL,dMatR);
	//makePointCloud();
	//findCorrespondences();
	//Ptr<IplImage> Image=&img;
/*if(MODE_KP_CALC==0)
{
	//CvMat image=img;
	//namedWindow( "Example1", CV_WINDOW_AUTOSIZE );
	calcDescriptors(img);
	//cvReleaseImage( &image );
	//cvDestroyWindow( "Example1" );
}
else
{
	imshow( "Input", img );
	vector<KeyPoint> kp=getKeypoints(img);
	vector<KeyPoint> kp1=getKeypoints(img);
}*/
cvWaitKey(0);
}
JNIEXPORT void JNICALL Java_vision_LoadImage_matchImage
  (JNIEnv *env, jclass cl, jstring left, jstring right)
{
	Mat leftImage = imread( env->GetStringUTFChars(left, JNI_FALSE) );
	Mat rightImage = imread( env->GetStringUTFChars(right, JNI_FALSE) );
	imshow("Left",leftImage);
	imshow("Right",rightImage);
	//rectifyImage(leftImage,rightImage);
	cvWaitKey(0);
}
