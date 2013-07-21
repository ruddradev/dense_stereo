#include "cv.h"
#include "cxmisc.h"
#include "highgui.h"
#include "cvaux.h"
#include "cvwimage.h"
#include "stereoCalibration.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <windows.h>
#include <ctype.h>

;using namespace cv;

	int xPatSize;
	int yPatSize;
	int n;

	CvMat* IMatRight;
	CvMat* DMatRight;
	CvMat* IMatLeft;
	CvMat* DMatLeft;
	CvMat* EssentialMat;
	CvMat* FundamentalMat;
	

void stereoCalib(int xPatSize,int yPatSize,int n,
				 CvMat* IMLeft,CvMat* IMRight,CvMat* FMat,CvMat* dMatL,CvMat* dMatR)
{
	IplImage* refImage;
	/*if(imageList ==NULL)
	{
		printf("imageList file cannot be read.");
			return;
	}*/
	int successes=0;
	CvSize patternSize= cvSize(xPatSize,yPatSize);
	int board_n=xPatSize*yPatSize;
	CvPoint2D32f* leftImageCorners = new CvPoint2D32f[ board_n ];
	CvPoint2D32f* rightImageCorners = new CvPoint2D32f[ board_n ];
	//Declare the parameter matirces
		CvMat* objectPoints=cvCreateMat(n*board_n,3,CV_32F);	
		CvMat* imagePointsLeft=cvCreateMat(n*board_n,2,CV_32F);
		CvMat* imagePointsRight=cvCreateMat(n*board_n,2,CV_32F);
		CvMat* pointCount=cvCreateMat(n,1,CV_32S);
		CvMat* intrinsicMatLeft=cvCreateMat(3,3,CV_64F);
		CvMat* intrinsicMatRight=cvCreateMat(3,3,CV_64F);
		CvMat* distortMatLeft=cvCreateMat(5,1,CV_64F);
		CvMat* distortMatRight=cvCreateMat(5,1,CV_64F);
		CvMat* rotationMat=cvCreateMat(3,3,CV_64F);
		CvMat* translationMat=cvCreateMat(3,1,CV_64F);
		CvMat* essentialMat=cvCreateMat(3,3,CV_64F);
		CvMat* fundamentalMat=cvCreateMat(3,3,CV_64F);

	//compute for each image
		/*for (int frames=0;frames<n;frames++)
		{
			Mat leftImage=imageList[frames][0];
			Mat rightImage=imageList[frames][1];*/
		CvCapture* leftCam = cvCreateCameraCapture( 1 );
		Sleep(100);
		CvCapture* rightCam = cvCreateCameraCapture( 2 );
		IplImage* leftImage;
		IplImage* rightImage;
		Mat leftImageMat;
		Mat rightImageMat;
		while(successes<n)
		{
			
			leftImage= cvQueryFrame( leftCam);
			rightImage= cvQueryFrame( rightCam);
			leftImageMat=Mat(leftImage);
			leftImageMat=Mat(leftImage);
			
			int leftPatFound = cvFindChessboardCorners(leftImage, patternSize, leftImageCorners,&board_n,
			CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
			+ CALIB_CB_FAST_CHECK);

			
			int rightPatFound = cvFindChessboardCorners(rightImage, patternSize, rightImageCorners,&board_n,
			CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
			+ CALIB_CB_FAST_CHECK);
			if(!(rightPatFound && leftPatFound))
			{
				printf("pattern %d corner extraction failed." ,successes);
				continue;
			}	
			printf("pattern %d corner extraction succeded.",successes);
			
/*0605*/	IplImage* grayLeftImage = cvCreateImage(cvSize(leftImage->width,leftImage->height),IPL_DEPTH_8U, 1);
			cvCvtColor(leftImage, grayLeftImage, CV_BGR2GRAY);
			
			cvFindCornerSubPix(grayLeftImage, leftImageCorners,board_n, Size(11, 11), Size(-1, -1),
			TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			cvDrawChessboardCorners(grayLeftImage,patternSize,leftImageCorners,board_n,leftPatFound);
			cvNamedWindow("Left subpix board points:");
			cvShowImage("Left subpix board points:", grayLeftImage);
			cvWaitKey(0);
			cvDestroyWindow("Left subpix board points:");

/*0605*/	IplImage* grayRightImage = cvCreateImage(cvSize(rightImage->width,rightImage->height),IPL_DEPTH_8U, 1);
			cvCvtColor(rightImage, grayRightImage, CV_BGR2GRAY);
			
			cvFindCornerSubPix(grayRightImage, rightImageCorners,board_n, Size(11, 11), Size(-1, -1),
			TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			cvDrawChessboardCorners(grayRightImage,patternSize,rightImageCorners,board_n,rightPatFound);
			
			cvNamedWindow("right subpix board points:");
			cvShowImage("right subpix board points:", grayRightImage);
			cvWaitKey(0);
			cvDestroyWindow("right subpix board points:");
			
			printf("Pattern %d successful out of %d",successes,n);
			
			int step = successes*board_n;
			for( int i=step, j=0; j<board_n; ++i,++j ) {
			
			CV_MAT_ELEM(*imagePointsLeft,float,i,0) = (float)leftImageCorners[j].x;
			CV_MAT_ELEM(*imagePointsLeft,float,i,1) = (float)leftImageCorners[j].y;
			CV_MAT_ELEM(*imagePointsRight,float,i,0) = (float)rightImageCorners[j].x;
			CV_MAT_ELEM(*imagePointsRight,float,i,1) = (float)rightImageCorners[j].y;
			CV_MAT_ELEM(*objectPoints,float,i,0) = (float)(j/xPatSize);
			CV_MAT_ELEM(*objectPoints,float,i,1) = (float)(j%xPatSize);
			CV_MAT_ELEM(*objectPoints,float,i,2) = 0.0f;
			}
			CV_MAT_ELEM(*pointCount,float,successes,0) = (float)board_n;
			//pause and hold mechanism
			int c = cvWaitKey(15);
			if(c == 'p'){
			c = 0;
			while(c != 'p' && c != 27){
			c = cvWaitKey(250);
			}
			}
			if(c == 27)
			return;
			successes++;
		}
			if(successes<=(n/2))
			{
				return;
			}
/*0705		CvMat* objectPoints2 = cvCreateMat(successes*board_n,3,CV_32FC3);
			CvMat* imagePointsLeft2 = cvCreateMat(successes*board_n,2,CV_32FC2);
			CvMat* imagePointsRight2 = cvCreateMat(successes*board_n,2,CV_32FC2);
			CvMat* pointCount2 = cvCreateMat(successes,1,CV_32SC1);
			imagePointsLeft2->rows = imagePointsLeft->rows=successes*board_n;

 				for(int i = 0; i<successes*board_n; ++i) {
				CV_MAT_ELEM( *imagePointsLeft2, float, i, 0)=CV_MAT_ELEM( *imagePointsLeft, float, i, 0);
				CV_MAT_ELEM( *imagePointsLeft2, float,i,1)=CV_MAT_ELEM( *imagePointsLeft, float, i, 1);
				CV_MAT_ELEM( *imagePointsRight2, float, i, 0)=CV_MAT_ELEM( *imagePointsRight, float, i, 0);
				CV_MAT_ELEM( *imagePointsRight2, float,i,1)=CV_MAT_ELEM( *imagePointsRight, float, i, 1);
				CV_MAT_ELEM(*objectPoints2, float, i, 0)=CV_MAT_ELEM( *objectPoints, float, i, 0) ;
				CV_MAT_ELEM( *objectPoints2, float, i, 1)=CV_MAT_ELEM( *objectPoints, float, i, 1) ;
				CV_MAT_ELEM( *objectPoints2, float, i, 2)=CV_MAT_ELEM( *objectPoints, float, i, 2) ;
				}
				for(int i=0; i<successes; ++i){ //These are all the same number
				CV_MAT_ELEM( *pointCount2, int, i, 0) =CV_MAT_ELEM( *pointCount, int, i, 0);
				}
				
				cvReleaseMat(&objectPoints);
				cvReleaseMat(&imagePointsLeft);
				cvReleaseMat(&imagePointsRight);
				cvReleaseMat(&pointCount);
				*/
				// At this point we have all of the chessboard corners we need.
				// Initialize the intrinsic matrix such that the two focal
				// lengths have a ratio of 1.0
				//
				CV_MAT_ELEM( *intrinsicMatLeft, float, 0, 0 ) = 1.0f;
				CV_MAT_ELEM( *intrinsicMatLeft, float, 1, 1 ) = 1.0f;
				CV_MAT_ELEM( *intrinsicMatRight, float, 0, 0 ) = 1.0f;
				CV_MAT_ELEM( *intrinsicMatRight, float, 1, 1 ) = 1.0f;
				//CvMat imageLeft=imageList[0][0];
				//CvMat imageRight=imageList[0][1];
				//CALIBRATE THE LEFT CAMERA!
				

	//0705		//converting to multichannel
				CvMat* _objectPoints=&cvMat(n*board_n,1,CV_32FC3);
				CvMat* _imagePointsLeft=&cvMat(n*board_n,1,CV_32FC2);
				CvMat* _imagePointsRight=&cvMat(n*board_n,1,CV_32FC2);
				
				cvSetIdentity(intrinsicMatLeft);
				cvSetIdentity(intrinsicMatRight);
				cvZero(distortMatLeft);
				cvZero(distortMatRight);
			cvReshape(objectPoints,_objectPoints,3,0);
			cvReshape(imagePointsLeft,_imagePointsLeft,2,0);
			cvReshape(imagePointsRight,_imagePointsRight,2,0);
			
			Beep(523,100);
			
/*0705Error Here!*//*	cvCalibrateCamera2(
				_objectPoints, _imagePointsLeft,				//add 2 to respective variables after uncommenting prev sec
				pointCount, cvGetSize(leftImage ),
				intrinsicMatLeft, distortMatLeft,
				NULL, NULL,0 //CV_CALIB_FIX_ASPECT_RATIO
				);
Beep(523,500);
				// SAVE THE INTRINSICS AND DISTORTIONS
				cvSave("IntrinsicsLeft.xml",intrinsicMatLeft);
				cvSave("DistortionLeft.xml",distortMatLeft);
				//CALIBRATE THE RIGHT CAMERA!
				cvCalibrateCamera2(
				_objectPoints, _imagePointsRight,
				_pointCount, cvGetSize(rightImage ),
				intrinsicMatRight, distortMatRight,
				NULL, NULL,0);
				// SAVE THE INTRINSICS AND DISTORTIONS
				cvSave("IntrinsicsRight.xml",intrinsicMatRight);
				cvSave("DistortionRight.xml",distortMatRight);
				cvSave("IntrinsicsLeft.xml",intrinsicMatLeft);
				cvSave("DistortionLeft.xml",distortMatLeft);
				cvSave("Fundamental.xml",fundamentalMat);
				*/
				//Stereo Calibration
			cvNamedWindow("LeftImage");
			cvShowImage("LeftImage",leftImage);
			cvWaitKey(0);
			cvDestroyWindow("LeftImage");
				cvStereoCalibrate( objectPoints, imagePointsLeft,
				imagePointsRight, pointCount,
				intrinsicMatLeft, distortMatLeft, intrinsicMatRight, distortMatRight,
				cvGetSize(leftImage), rotationMat, translationMat, essentialMat, fundamentalMat,
				cvTermCriteria(CV_TERMCRIT_ITER+
				CV_TERMCRIT_EPS, 100, 1e-5),
				CV_CALIB_FIX_ASPECT_RATIO +
				CV_CALIB_ZERO_TANGENT_DIST +
				CV_CALIB_SAME_FOCAL_LENGTH );
				
				// EMat=essentialMat;
				FMat=fundamentalMat;
				dMatL=distortMatLeft;
				dMatR=distortMatRight;
				//

}
void rectifyImage(Mat left, Mat right)
{
	int xPatSize=9;
 int yPatSize=6;
 int n=15;
	
	if(IMatRight==NULL || DMatRight==NULL || IMatLeft==NULL || DMatLeft==NULL ||FundamentalMat==NULL)
	{
		IMatRight= (CvMat*)cvLoad("IntrinsicsRight.xml");
		DMatRight= (CvMat*)cvLoad("DistortionRight.xml");
		IMatLeft= (CvMat*)cvLoad("IntrinsicsLeft.xml");
		DMatLeft= (CvMat*)cvLoad("DistortionLeft.xml");
		 //EssentialMat=(CvMat*)cvLoad("Essential.xml");
		 FundamentalMat= (CvMat*)cvLoad("Fundamental.xml");
		 if(IMatRight==NULL || DMatRight==NULL || IMatLeft==NULL || DMatLeft==NULL ||FundamentalMat==NULL)
		{
		printf("Please wait. Calibrating....");
		stereoCalib(xPatSize,yPatSize,n,IMatLeft,IMatRight,FundamentalMat,DMatLeft,DMatRight);
		 }
	}
	
}