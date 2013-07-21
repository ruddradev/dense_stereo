//#include <jni.h>
//#include <vision_LoadImage.h>
#include <cvaux.h>
#include <highgui.h>
//IplImage* img;
int MODE_KP_CALC=0;

int nImage=15;
int board_n=9*6;
		CvMat* objectPoints=cvCreateMat(nImage*board_n,3,CV_32FC1);	
		CvMat* imagePointsLeft=cvCreateMat(nImage*board_n,2,CV_32FC1);
		CvMat* imagePointsRight=cvCreateMat(nImage*board_n,2,CV_32FC1);
		CvMat* pointCount=cvCreateMat(nImage,1,CV_32SC1);
		CvMat* IMLeft=cvCreateMat(3,3,CV_32FC1);
		CvMat* IMRight=cvCreateMat(3,3,CV_32FC1);
		CvMat* dMatL=cvCreateMat(5,1,CV_32FC1);
		CvMat* dMatR=cvCreateMat(5,1,CV_32FC1);
		CvMat* rotationMat=cvCreateMat(3,3,CV_32FC1);
		CvMat* translationMat=cvCreateMat(3,1,CV_32FC1);
		CvMat* EMat=cvCreateMat(3,3,CV_32FC1);
		CvMat* FMat=cvCreateMat(3,3,CV_32FC1);

/*JNIEXPORT void JNICALL Java_vision_LoadImage_loadImage
  (JNIEnv *env, jclass cl, jstring fname);


JNIEXPORT void JNICALL Java_vision_LoadImage_matchImage
  (JNIEnv *env, jclass cl, jstring left, jstring right);*/