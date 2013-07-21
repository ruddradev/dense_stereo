#pragma warning( disable: 4996 )
/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warrenty, support or any guarentee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008
 
   AVAILABLE AT: 
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130    

   OTHER OPENCV SITES:
   * The source code is on sourceforge at:
     http://sourceforge.net/projects/opencvlibrary/
   * The OpenCV wiki page (As of Oct 1, 2008 this is down for changing over servers, but should come back):
     http://opencvlibrary.sourceforge.net/
   * An active user group is at:
     http://tech.groups.yahoo.com/group/OpenCV/
   * The minutes of weekly OpenCV development meetings are at:
     http://pr.willowgarage.com/wiki/OpenCV
   ************************************************** */

/*
	Modified by Martin Peris Martorell (info@martinperis.com) in order to accept some configuration
	parameters and store all the calibration data as xml files.

*/

#include <cv.h>
#include <cxmisc.h>
#include <highgui.h>
#include <cvaux.h>
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>
//#include "stereo1.h"
using namespace std;
using namespace cv;
CvCapture* leftCam;
CvCapture* rightCam;
IplImage* imagePair[2];
IplImage* imagePairs[2][15];
CvStereoBMState *BMStateTuned;
int picIndex;
int succMatches;
//
// Given a list of chessboard images, the number of corners (nx, ny)
// on the chessboards, and a flag: useCalibrated for calibrated (0) or
// uncalibrated (1: use cvStereoCalibrate(), 2: compute fundamental
// matrix separately) stereo. Calibrate the cameras and display the
// rectified results along with the computed disparity images.
//
void convertQMatrix()
{
	CvMat* q= (CvMat*)cvLoad("C:\\Camera_calib_data\\Q.xml");
	cvSave("C:\\Camera_calib_data\\Q.txt",q);

}

void capImage(int max=15)
{
	Beep(1000,500);
char numstr[3];
for(int i=0;i<max;i++)
{
	//imagePairs[0][i]=cvQueryFrame(leftCam);
	//imagePairs[1][i]=cvQueryFrame(rightCam);
	imagePairs[0][i]=&IplImage(imread("C:\\mimg\\stereo\\"+ string(itoa((2*i),numstr,10))+".jpg"));
	imagePairs[1][i]=&IplImage(imread("C:\\mimg\\stereo\\"+ string(itoa(((2*i)+1),numstr,10))+".jpg"));
	cvNamedWindow("left");
	cvNamedWindow("right");
	cvShowImage("left",imagePairs[0][i]);
	cvShowImage("right",imagePairs[1][i]);
	Beep(500,400);
	Sleep(1000);
}
cvDestroyWindow("left");
cvDestroyWindow("right");
}
void initCameras()
{
	leftCam=cvCreateCameraCapture(2);
	Sleep(100);
	rightCam=cvCreateCameraCapture(1);
	printf("Done initCameras");
	fflush(stdout);
	Sleep(200);
}
void changeImagePair()
{
	if(picIndex==15)
	{
		Beep(1000,500);
		Sleep(300);
		Beep(1000,500);
		int max=15-succMatches;
		capImage(max);
		picIndex=0;
	}
	imagePair[0]=imagePairs[0][picIndex];
	imagePair[1]=imagePairs[1][picIndex];
	picIndex++;
	  //imagePair[0]=cvQueryFrame(leftCam);
	  //imagePair[1]=cvQueryFrame(rightCam);
	/*cvNamedWindow("left");
	cvShowImage("left",imagePair[0]);
	cvNamedWindow("right");
	cvShowImage("right",imagePair[1]);*/
	//imagePair[0]=cvLoadImage("C:\\mimg\\left.jpg");
	//imagePair[1]=cvLoadImage("C:\\mimg\\right.jpg");
	//imwrite("C:\\mimg\\left.jpg",Mat(imagePair[0]));
	//imwrite("C:\\mimg\\right.jpg",Mat(imagePair[1]));
}
void alignCameras()
{
	while(true)
	{
		changeImagePair();
		IplImage* left=imagePair[0];
		IplImage* right=imagePair[1];
		CvSize imgSize=cvGetSize(left);
		CvPoint one= cvPoint((imgSize.width/2)-50,imgSize.height/2);
		CvPoint two= cvPoint((imgSize.width/2)+50,imgSize.height/2);
		CvPoint three= cvPoint((imgSize.width/2),imgSize.height/2-50);
		CvPoint four= cvPoint((imgSize.width/2),imgSize.height/2+50);
		CvScalar color= cvScalar(255,0,0);
		cvLine(left,one,two,color);
		cvLine(left,three,four,color);
		cvLine(right,one,two,color);
		cvLine(right,three,four,color);
		cvNamedWindow("left");
	cvNamedWindow("right");
	cvShowImage("left",left);
	cvShowImage("right",right);
	char c=cvWaitKey(10);
    if(c==27)
		break;
	}
	cvDestroyWindow("left");
	cvDestroyWindow("right");
}
void makeBMtune()
{
		int preFilterSize=3;
		int preFilterCap=63;
		int SADWindowSize=5;
		int minDisparity=5;
		int numberOfDisparities=7;
		int textureThreshold=200;
		int uniquenessRatio=0;
	cvNamedWindow("BMTune",1);
		cvCreateTrackbar("preFilterSize","BMTune",&preFilterSize,200);
		cvCreateTrackbar("preFilterCap","BMTune",&preFilterCap,63);
		cvCreateTrackbar("SADWindowSize","BMTune",&SADWindowSize,200);
		cvCreateTrackbar("minDisparity","BMTune",&minDisparity,200);
		cvCreateTrackbar("numberOfDisparities","BMTune",&numberOfDisparities,15);
		cvCreateTrackbar("textureThreshold","BMTune",&textureThreshold,200);
		cvCreateTrackbar("uniquenessRatio","BMTune",&uniquenessRatio,200);
}
void setBMtune()
{
	int preFilterSize=cvGetTrackbarPos("preFilterSize","BMTune");
		int preFilterCap=cvGetTrackbarPos("preFilterCap","BMTune");
		int SADWindowSize=cvGetTrackbarPos("SADWindowSize","BMTune");
		int minDisparity=cvGetTrackbarPos("minDisparity","BMTune");
		int numberOfDisparities=cvGetTrackbarPos("numberOfDisparities","BMTune");
		int textureThreshold=cvGetTrackbarPos("textureThreshold","BMTune");
		int uniquenessRatio=cvGetTrackbarPos("uniquenessRatio","BMTune");
		if(preFilterSize<5)
			preFilterSize=5;
	if(preFilterSize%2==0)
	preFilterSize++;
	if(preFilterCap%2==0)
	preFilterCap++;
	if(SADWindowSize<5)
	SADWindowSize=5;
	if(SADWindowSize%2==0)
	SADWindowSize++;
	BMStateTuned=cvCreateStereoBMState();
	BMStateTuned->preFilterSize=preFilterSize;//41;
        BMStateTuned->preFilterCap=preFilterCap;//31;
        BMStateTuned->SADWindowSize=SADWindowSize;//41;
        BMStateTuned->minDisparity=(-1)*minDisparity; //Default -64
        BMStateTuned->numberOfDisparities=numberOfDisparities*16;//128
        BMStateTuned->textureThreshold=textureThreshold;//15
        BMStateTuned->uniquenessRatio=uniquenessRatio;//15;

}
void StereoCalib(const char* imageList, int nx, int ny 
				 ,int useUncalibrated, float _squareSize,int isCalibrated)
{
	initCameras();
	//capImage();
	picIndex=0;
    int displayCorners = 1;
    int showUndistorted = 1;
    bool isVerticalStereo = false;//OpenCV can handle left-right
                                      //or up-down camera arrangements
    const int maxScale = 3;
    const float squareSize = _squareSize; //Chessboard square size in cm
    //FILE* f = fopen(imageList, "rt");
    int i, j,k, lr, n = nx*ny, N = 0;
	int nframes=0;
    vector<string> imageNames[2];
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<CvPoint2D32f> temp(n);
    CvSize imageSize = {640,480};
    // ARRAY AND VECTOR STORAGE:
    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];
    double Q[4][4];
    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
    CvMat _R = cvMat(3, 3, CV_64F, R );
    CvMat _T = cvMat(3, 1, CV_64F, T );
    CvMat _E = cvMat(3, 3, CV_64F, E );
    CvMat _F = cvMat(3, 3, CV_64F, F );
    CvMat _Q = cvMat(4,4, CV_64F, Q);
	CvMat* mx1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* my1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* mx2 = cvCreateMat( imageSize.height,

            imageSize.width, CV_32F );
        CvMat* my2 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* img1r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* img2r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* pair;
        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
        CvMat _R2 = cvMat(3, 3, CV_64F, R2);
		CvMat* _nframes;
		if(isCalibrated==2)
		{
			alignCameras();
			isCalibrated=0;
		}
Beep(1000,500);
	if(isCalibrated==1)
	{
	CvMat *_M1=(CvMat*)cvLoad("D:\\Camera_calib_data\\M1.xml");
	CvMat *_M2=(CvMat*)cvLoad("D:\\Camera_calib_data\\M2.xml");
	CvMat *_D1=(CvMat*)cvLoad("D:\\Camera_calib_data\\D1.xml");
	CvMat *_D2=(CvMat*)cvLoad("D:\\Camera_calib_data\\D2.xml");
	CvMat *_E=(CvMat*)cvLoad("D:\\Camera_calib_data\\E.xml");
	CvMat *_F=(CvMat*)cvLoad("D:\\Camera_calib_data\\F.xml");
	//CvMat *_Q=(CvMat*)cvLoad("D:\\Camera_calib_data\\Q.xml");
	CvMat *_R2=(CvMat*)cvLoad("D:\\Camera_calib_data\\R2.xml");
	CvMat *_R1=(CvMat*)cvLoad("D:\\Camera_calib_data\\R1.xml");
     mx1=(CvMat*)cvLoad("D:\\Camera_calib_data\\mx1.xml");
     my1=(CvMat*)cvLoad("D:\\Camera_calib_data\\my1.xml");
     mx2=(CvMat*)cvLoad("D:\\Camera_calib_data\\mx2.xml");
     my2=(CvMat*)cvLoad("D:\\Camera_calib_data\\my2.xml");
	 _nframes=(CvMat*)cvLoad("D:\\Camera_calib_data\\nframes.xml");
	}
    if( displayCorners )
        cvNamedWindow( "corners", 1 );
// READ IN THE LIST OF CHESSBOARDS:
    /*if( !f )
    {
        fprintf(stderr, "can not open file %s\n", imageList );
        return;
    }*/
	int gotCorners=2;
    for(i=0;i<30;)
    {
		succMatches=i;
		if(i==47)
			Beep(1500,500);
		printf("\niteration %d\n",i);
		if(isCalibrated==1)
			break;
		if(isCalibrated==0)
		{
		Beep(500,200);
		Sleep(1000);
		}
		if(gotCorners==2&& isCalibrated==0)
		{
			changeImagePair();
			i++;
		}
		else if(gotCorners==1&& isCalibrated==0)
		{
			printf("Second Image unacceptable.. Pair deleted\n");
			fflush(stdout);
			active[0].erase(active[0].end()-1);
			printf("active[0].end erased\n");
			fflush(stdout);
			points[0].erase(points[0].end()-(n+1),points[0].end()-1);
			changeImagePair();
		}
		else if (gotCorners==0 && isCalibrated==0)
		{
			printf("First Image unacceptable.. Pair deleted\n");
			fflush(stdout);
			changeImagePair();
		}
		gotCorners=2;
		for(k=0;k<=1;k++)
		{
        //char buf[1024];
        int count = 0, result=0;
        //lr = i % 2;
        vector<CvPoint2D32f>& pts = points[k];
        //if( !fgets( buf, sizeof(buf)-3, f ))
          //  break;
        //size_t len = strlen(buf);
        //while( len > 0 && isspace(buf[len-1]))
          //  buf[--len] = '\0';
        //if( buf[0] == '#')
          //  continue;
		IplImage* img =imagePair[k];
        //IplImage* img =imagePairs[k][i];
		//IplImage* img = cvLoadImage( buf, 0 );
        if( !img )
            break;
        imageSize = cvGetSize(img);
        //imageNames[lr].push_back(buf);
		/*cvNamedWindow("Img");
		cvShowImage("Img",img);
		cvWaitKey();*/
if(isCalibrated==0)
{
    //FIND CHESSBOARDS AND CORNERS THEREIN:
        for(int s = 1; s <= maxScale; s++ )
        {
            IplImage* timg = img;
            if( s > 1 )
            {
                timg = cvCreateImage(cvSize(img->width*s,img->height*s),
                    img->depth, img->nChannels );
                cvResize( img, timg, CV_INTER_CUBIC );
            }
            result = cvFindChessboardCorners( timg, cvSize(nx, ny),
                &temp[0], &count,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            if( timg != img ) 
                cvReleaseImage( &timg );
            if( result || s == maxScale )
                for( j = 0; j < count; j++ )
            {
                temp[j].x /= s;
                temp[j].y /= s;
            }
            if( result )
                break;
        }
		 printf("%d\n",result);
			fflush(stdout);
		if(!result)
		{
			gotCorners=k;
			break;
		}

        if( displayCorners )
        {
           
            IplImage* cimg = img;
            //cvCvtColor( img, cimg, CV_GRAY2BGR );
            cvDrawChessboardCorners( cimg, cvSize(nx, ny), &temp[0],
                count, result );
            cvShowImage( "corners", cimg );
            //cvReleaseImage( &cimg );
			

            if( cvWaitKey(1) == 27 ) //Allow ESC to quit
                exit(-1);
        }
        else
            putchar('.');
		
        N = pts.size();
        pts.resize(N + n, cvPoint2D32f(0,0));
        active[k].push_back((uchar)result);
    //assert( result!=0);
        if( result )
        {
         //Calibration will suffer without subpixel interpolation
			IplImage* gimg = cvCreateImage( imageSize, 8, 1 );
			cvCvtColor( img, gimg, CV_RGB2GRAY);
            cvFindCornerSubPix( gimg, &temp[0], count,
                cvSize(11, 11), cvSize(-1,-1),
                cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                30, 0.01) );
            copy( temp.begin(), temp.end(), pts.begin() + N );
			cvReleaseImage( &gimg );
        }
        //cvReleaseImage( &img );
}// k loop
    }

	}
    //fclose(f);
    printf("\n");
	Beep(900,500);
// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
    if(isCalibrated==0)
	{
	nframes = active[0].size();//Number of good chessboads found
	_nframes=&cvMat(1,1,CV_32S,&nframes);
	cvSave("D:\\Camera_calib_data\\nframes.xml",_nframes);
	objectPoints.resize(nframes*n);
	for( i = 0; i < ny; i++ )
        for( j = 0; j < nx; j++ )
        objectPoints[i*nx + j] = cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < nframes; i++ )
        copy( objectPoints.begin(), objectPoints.begin() + n,
        objectPoints.begin() + i*n );
    npoints.resize(nframes,n);
    N = nframes*n;
    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
	CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
	CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
    cvSetIdentity(&_M1);
    cvSetIdentity(&_M2);
    cvZero(&_D1);
    cvZero(&_D2);

	cvDestroyWindow("corners");
// CALIBRATE THE STEREO CAMERAS
    printf("Running stereo calibration ...");
    fflush(stdout);
    cvStereoCalibrate( &_objectPoints, &_imagePoints1,
        &_imagePoints2, &_npoints,
        &_M1, &_D1, &_M2, &_D2,
        imageSize, &_R, &_T, &_E, &_F,
        cvTermCriteria(CV_TERMCRIT_ITER+
        CV_TERMCRIT_EPS, 100, 1e-5),
        CV_CALIB_FIX_ASPECT_RATIO +
        CV_CALIB_ZERO_TANGENT_DIST +
        CV_CALIB_SAME_FOCAL_LENGTH );
    printf(" done\n");
	fflush(stdout);
// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    vector<CvPoint3D32f> lines[2];
    points[0].resize(N);
    points[1].resize(N);
    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    lines[0].resize(N);
    lines[1].resize(N);
    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
//Always work in undistorted space
    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
        &_M1, &_D1, 0, &_M1 );
    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
        &_M2, &_D2, 0, &_M2 );
    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
    double avgErr = 0;
    for( i = 0; i < N; i++ )
    {
        double err = fabs(points[0][i].x*lines[1][i].x +
            points[0][i].y*lines[1][i].y + lines[1][i].z)
            + fabs(points[1][i].x*lines[0][i].x +
            points[1][i].y*lines[0][i].y + lines[0][i].z);
        avgErr += err;
    }
    printf( "avg err = %g\n", avgErr/(nframes*n) );
	fflush(stdout);
            cvSave("D:\\Camera_calib_data\\D1.xml",&_D1);
            cvSave("D:\\Camera_calib_data\\D2.xml",&_D2);
            cvSave("D:\\Camera_calib_data\\Q.xml",&_Q);
	// end of the calibration block
//COMPUTE AND DISPLAY RECTIFICATION
    if( showUndistorted )
    {
        
// IF BY CALIBRATED (BOUGUET'S METHOD)
        if( useUncalibrated == 0 )
        {
            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
            cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
                &_R, &_T,
                &_R1, &_R2, &_P1, &_P2, &_Q,
                0/*CV_CALIB_ZERO_DISPARITY*/ );
            isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
			double avgErr = 0;
    for( i = 0; i < N; i++ )
    {
        double err = fabs(points[0][i].x*lines[1][i].x +
            points[0][i].y*lines[1][i].y + lines[1][i].z)
            + fabs(points[1][i].x*lines[0][i].x +
            points[1][i].y*lines[0][i].y + lines[0][i].z);
        avgErr += err;
    }
    printf( "avg err = %g\n", avgErr/(nframes*n) );
	fflush(stdout);
    //Precompute maps for cvRemap()
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
            cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
            
    //Save parameters
			cvSave("D:\\Camera_calib_data\\P1.xml",&_P1);
			cvSave("D:\\Camera_calib_data\\P2.xml",&_P2);
			cvSave("D:\\Camera_calib_data\\mx1.xml",mx1);
			cvSave("D:\\Camera_calib_data\\my1.xml",my1);
			cvSave("D:\\Camera_calib_data\\mx2.xml",mx2);
			cvSave("D:\\Camera_calib_data\\my2.xml",my2);
			
        }
//OR ELSE HARTLEY'S METHOD
        else if( useUncalibrated == 1 || useUncalibrated == 2 )
     // use intrinsic parameters of each camera, but
     // compute the rectification transformation directly
     // from the fundamental matrix
        {

            double H1[3][3], H2[3][3], iM[3][3];
            CvMat _H1 = cvMat(3, 3, CV_64F, H1);
            CvMat _H2 = cvMat(3, 3, CV_64F, H2);
            CvMat _iM = cvMat(3, 3, CV_64F, iM);
    //Just to show you could have independently used F
            if( useUncalibrated == 2 )
                cvFindFundamentalMat( &_imagePoints1,
                &_imagePoints2, &_F);
            cvStereoRectifyUncalibrated( &_imagePoints1,
                &_imagePoints2, &_F,
                imageSize,
                &_H1, &_H2, 3);
            cvInvert(&_M1, &_iM);
            cvMatMul(&_H1, &_M1, &_R1);
            cvMatMul(&_iM, &_R1, &_R1);
            cvInvert(&_M2, &_iM);
            cvMatMul(&_H2, &_M2, &_R2);
            cvMatMul(&_iM, &_R2, &_R2);
			
    //Precompute map for cvRemap()
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_M1,mx1,my1);
            cvInitUndistortRectifyMap(&_M2,&_D1,&_R2,&_M2,mx2,my2);
			//Beep(500,1000);
			cvSave("D:\\Camera_calib_data\\R1.xml",&_R1);
			cvSave("D:\\Camera_calib_data\\R2.xml",&_R2);
			cvSave("D:\\Camera_calib_data\\M1.xml",&_M1);
			cvSave("D:\\Camera_calib_data\\M2.xml",&_M2);
			cvSave("D:\\Camera_calib_data\\mx1.xml",mx1);
			cvSave("D:\\Camera_calib_data\\my1.xml",my1);
			cvSave("D:\\Camera_calib_data\\mx2.xml",mx2);
			cvSave("D:\\Camera_calib_data\\my2.xml",my2);
			
		}
        else
            assert(0);
	}
	}
        //cvNamedWindow( "rectified", 1 );
// RECTIFY THE IMAGES AND FIND DISPARITY MAPS
     /*   if( !isVerticalStereo )
            pair = cvCreateMat( imageSize.height, imageSize.width*2,
            CV_8UC3 );
        else
            pair = cvCreateMat( imageSize.height*2, imageSize.width,
            CV_8UC3 );*/
//Setup for finding stereo corrrespondences
        CvStereoBMState *BMState = cvCreateStereoBMState();
        assert(BMState != 0);
        BMState->preFilterSize=9;//41;
        BMState->preFilterCap=31;//31;
        BMState->SADWindowSize=9;//41;
        BMState->minDisparity=0; //Default -64
        BMState->numberOfDisparities=32;//128
        BMState->textureThreshold=53;//15
        BMState->uniquenessRatio=0;//15;
		//GC state

		CvStereoGCState *GCState=cvCreateStereoGCState(128,5); 
		// Define BM tuning window
		makeBMtune();
		setBMtune();
        clock_t start=clock();
		clock_t stop;
		printf("BMState done");
		fflush(stdout);
			CvMat* disp = cvCreateMat( imageSize.height,
            imageSize.width, CV_16S );
			CvMat* dispr = cvCreateMat( imageSize.height,
            imageSize.width, CV_16S );
        CvMat* vdisp = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
		if(isCalibrated==1)
		{
			//nframes=30;
			/*nframes= CV_MAT_ELEM(*_nframes,int,0,0);
			printf("nframes=%d\n",nframes);
			fflush(stdout);*/
		}
		cvNamedWindow( "disparity",1 );
		bool pause=false;
		for( i = 0;;i++)
        {
			setBMtune();
			if(pause==false)
			changeImagePair();
            IplImage* img1=imagePair[0];
            IplImage* img2=imagePair[1];
			//Beep(500,500);
			imageSize=cvGetSize(img1);
			IplImage* gimg1 = cvCreateImage( imageSize, 8, 1 );
			cvCvtColor( img1, gimg1, CV_RGB2GRAY);
			IplImage* gimg2 = cvCreateImage( imageSize, 8, 1 );
			cvCvtColor( img2, gimg2, CV_RGB2GRAY);
            if( gimg1 && gimg2 )
            {
                CvMat part;
				cvRemap( gimg1, img1r, mx1, my1 );
                cvRemap( gimg2, img2r, mx2, my2 );
                if( !isVerticalStereo || useUncalibrated != 0 )
                {
              
                    cvFindStereoCorrespondenceBM(img1r,img2r, disp,
                        BMStateTuned);
                    cvNormalize(disp, vdisp, 0, 256, CV_MINMAX );
                    cvShowImage( "disparity", vdisp);
					cvSave("D:\\maps\\disp.txt",disp);
                }
				if(cvWaitKey(1)=='p')
					pause=true;
				if(cvWaitKey(1)=='c')
					pause=false;
                if( cvWaitKey(1) == 27 )
                    break;
            
				stop=clock();
				printf("%f\n",(float)(stop-start));
				start=stop;

			}
            
        }
		/*cvReleaseImage( &img1 );
            cvReleaseImage( &img2 );*/
        cvReleaseStereoBMState(&BMState);
        cvReleaseMat( &mx1 );
        cvReleaseMat( &my1 );
        cvReleaseMat( &mx2 );
        cvReleaseMat( &my2 );
        cvReleaseMat( &img1r );
        cvReleaseMat( &img2r );
        cvReleaseMat( &disp );
    
}

