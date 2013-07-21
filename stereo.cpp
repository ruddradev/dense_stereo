#include "cv.h"
#include "cxmisc.h"
#include "highgui.h"
#include "cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>
using namespace std;
//
// Given a list of chessboard images, the number of corners (nx, ny)
// on the chessboards, and a flag called useCalibrated (0 for Hartley
// or 1 for Bouguet stereo methods). Calibrate the cameras and display the
// rectified results along with the computed disparity images.
//
void makeList(vector<IplImage*> imageList,int n)
{
		CvCapture* leftCam = cvCreateCameraCapture( 1 );
		Sleep(100);
		CvCapture* rightCam = cvCreateCameraCapture( 2 );
		IplImage* leftImage;
		IplImage* rightImage;
		int lr=0;
		int counter=0;
		//vector<vector<IplImage*>> imageList1=*imageList;
		while(counter<n)
		{
			//populating a vector
			leftImage= cvQueryFrame( leftCam);
			rightImage= cvQueryFrame( rightCam);
			imageList.push_back(leftImage);
			imageList.push_back(rightImage);
			/*imageList[counter][0]=leftImage;
			imageList[counter][1]=rightImage;*/
			Beep(700,100);
			counter+=2;
		}
		// Images Loaded Correctly. Check using this block.
		for(int i=0;i<n;i++)
		{
		cvNamedWindow("Demo",1);
		cvShowImage("Demo",imageList[(i/2)]);
		cvNamedWindow("Demosecond",1);
		cvShowImage("Demosecond",imageList[(i/2)+1]);
		cvWaitKey(0);
		}
		
}
void StereoCalib(/*const char* imageList,*/int n_frames, int nx, int ny, int useUncalibrated)
{
int displayCorners = 1;
int showUndistorted = 1;
bool isVerticalStereo = false;//OpenCV can handle left-right
n_frames=15;
//or up-down camera arrangements
const int maxScale = 1;
const float squareSize = 1.f; //Set this to your actual square size
//0805   FILE* f = fopen(imageList, "rt");
int i, j, lr, nframes, n = nx*ny, N = 0;
CvSize imageSize = {0,0};
//Making an image List
//vector<IplImage*> imageList(30);
//makeList(imageList,n_frames);
CvCapture* leftCam = cvCreateCameraCapture( 1 );
Sleep(100);
CvCapture* rightCam = cvCreateCameraCapture( 2 );
//cvNamedWindow("Demo",1);
IplImage* Demo= cvQueryFrame(leftCam);
/*cvShowImage("Demo",Demo);
cvWaitKey(0);*/
imageSize = cvGetSize(Demo);
//vector<string> imageNames[2];
vector<CvPoint3D32f> objectPoints;
vector<CvPoint2D32f> points[2];
vector<int> npoints;
vector<uchar> active[2];
vector<CvPoint2D32f> temp(n);
//Beep(523,500);
// ARRAY AND VECTOR STORAGE:
double M1[3][3], M2[3][3], D1[5], D2[5];
double R[3][3], T[3], E[3][3], F[3][3];
CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
CvMat _R = cvMat(3, 3, CV_64F, R );
CvMat _T = cvMat(3, 1, CV_64F, T );
CvMat _E = cvMat(3, 3, CV_64F, E );
CvMat _F = cvMat(3, 3, CV_64F, F );
if( displayCorners )
cvNamedWindow( "corners", 1 );
// READ IN THE LIST OF CHESSBOARDS:
/*0805   if( !f )
{
fprintf(stderr, "can not open file %s\n", imageList );
return;
}*/
IplImage* img[2];
for(i=0;;i++)
{
int count = 0, result=0;
lr = i % 2;  //
//int index=i/2;
//imagePair=(vector<IplImage*>)imageList[index];
img[0] =cvQueryFrame(leftCam);
img[1]=cvQueryFrame(rightCam);
Beep(1500,500);
/*cvNamedWindow("img",1);
cvShowImage("img",img);
cvWaitKey(0);
cvDestroyWindow("img");*/
if(!img[0]|| !img[1] || i>=n_frames)
break;
for(int k=0;k<=1;k++)
{
vector<CvPoint2D32f>& pts = points[k];
//imageNames[lr].push_back(buf);
//FIND CHESSBOARDS AND CORNERS THEREIN:
for( int s = 1; s <= maxScale; s++ )
{
IplImage* timg = img[k];
if( s > 1 )
{
timg = cvCreateImage(cvSize(img[k]->width*s,img[k]->height*s),
img[k]->depth, img[k]->nChannels );
cvResize( img[k], timg, CV_INTER_CUBIC );
}
result = cvFindChessboardCorners( timg, cvSize(nx, ny),
&temp[0], &count,CV_CALIB_CB_ADAPTIVE_THRESH |CV_CALIB_CB_FILTER_QUADS);
//check if timg is loaded properly  20:16,0905, image [1][0] not loaded
/*cvNamedWindow("timg",1);
cvShowImage("timg",timg);
cvWaitKey(0);
cvDestroyWindow("timg");*/
if( timg != img[k] )
cvReleaseImage( &timg );
if( result || s == maxScale )
for( j = 0; j < count; j++ )
{
temp[j].x /= s;
temp[j].y /= s;
}
//Beep(1000,100);
if( result )
break;
}
if( displayCorners )
{
//printf("%s\n", buf);
cvDrawChessboardCorners( img[k], cvSize(nx, ny), &temp[0],
count, result );
cvShowImage( "corners", img[k] );
cvWaitKey(0);
//Beep(1000,100);
}
else
//putchar('.');
N = pts.size();
//pts.resize(N + n);
pts.resize(N + n, cvPoint2D32f(0,0));
active[k].push_back((uchar)result);
//assert( result != 0 );
if( result )
{
//Calibration will suffer without subpixel interpolation
IplImage* gimg = cvCreateImage( imageSize, 8, 1 );
cvCvtColor( img[k], gimg, CV_RGB2GRAY);

cvFindCornerSubPix( gimg, &temp[0], count,
cvSize(11, 11), cvSize(-1,-1),
cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
30, 0.01) );
copy( temp.begin(), temp.end(), pts.begin() + N );
//
//The image points are correctly stored in points[camNumber][pointIndex]
CvPoint2D32f pnts= temp[count-1];
printf("Temp image pts,count %d %f %f\n",count,pnts.x,pnts.y);
fflush(stdout);
//
cvReleaseImage( &gimg );
}
//Beep(1000,500);
}
}
//check block
CvPoint2D32f pnts= points[1][N/2];
printf("test image pts %f %f\n",pnts.x,pnts.y);
fflush(stdout);
//
cvReleaseImage( &img[0] );
cvReleaseImage( &img[1] );
//Beep(500,1000);
//fclose(f);
//printf("\n");
// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
nframes = active[0].size();//Number of good chessboads found
printf("%d\n",nframes);
fflush(stdout);
objectPoints.resize(nframes*n);
for( i = 0; i < ny; i++ )
for( j = 0; j < nx; j++ )
objectPoints[i*nx + j] =
cvPoint3D32f(i*squareSize, j*squareSize, 0);
//check object points
CvPoint3D32f objpnts= objectPoints[(int)(nx*ny)/2];
printf("test object pts %f %f %f\n",objpnts.x,objpnts.y,objpnts.z);
fflush(stdout);
//
for( i = 1; i < nframes; i++ )
copy( objectPoints.begin(), objectPoints.begin() + n,
objectPoints.begin() + i*n );
npoints.resize(nframes,n);
N = nframes*n;
CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
//check final object points
CvScalar opoints=cvGet2D(&_objectPoints,0,N/2);
printf("Objectpoints %f,%f\n", opoints.val[0],opoints.val[1]);
fflush(stdout);
//
//check final image points
CvScalar ipoints=cvGet2D(&_imagePoints1,0,N/2);
printf("Imagepoints %f,%f\n", ipoints.val[0],ipoints.val[1]);
fflush(stdout);
//

cvSetIdentity(&_M1);
cvSetIdentity(&_M2);
cvZero(&_D1);
cvZero(&_D2);
// CALIBRATE THE STEREO CAMERAS
printf("Running stereo calibration ...");
fflush(stdout);
cvStereoCalibrate( &_objectPoints, &_imagePoints1,
&_imagePoints2, &_npoints,
&_M1, &_D1, &_M2, &_D2,
imageSize, &_R, &_T, &_E, &_F,
cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),CV_CALIB_FIX_ASPECT_RATIO +CV_CALIB_ZERO_TANGENT_DIST +CV_CALIB_SAME_FOCAL_LENGTH );
printf(" done\n");
fflush(stdout);
cvSave("D:\\FundamentalMat.xml",&_F);
cvSave("D:\\DistortionLeft.xml",&_D1);
cvSave("D:\\DistortionRight.xml",&_D2);
cvSave("D:\\IntrinsicLeft.xml",&_M1);
cvSave("D:\\IntrinsicRight.xml",&_M2);
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
cvUndistortPoints( &_imagePoints1, &_imagePoints1,&_M1, &_D1, 0, &_M1 );
cvUndistortPoints( &_imagePoints2, &_imagePoints2,&_M2, &_D2, 0, &_M2 );
cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
double avgErr = 0;
for( i = 0; i < N; i++ )
{
double err = fabs(points[0][i].x*lines[1][i].x +points[0][i].y*lines[1][i].y + lines[1][i].z)+ fabs(points[1][i].x*lines[0][i].x +points[1][i].y*lines[0][i].y + lines[0][i].z);
avgErr += err;
}
printf( "avg err = %g\n", avgErr/(nframes*n) );
fflush(stdout);
//COMPUTE AND DISPLAY RECTIFICATION
if( showUndistorted )
{
CvMat* mx1 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
CvMat* my1 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
CvMat* mx2 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
CvMat* my2 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
CvMat* img1r = cvCreateMat( imageSize.height,imageSize.width, CV_8U );
CvMat* img2r = cvCreateMat( imageSize.height,imageSize.width, CV_8U );
CvMat* disp = cvCreateMat( imageSize.height,imageSize.width, CV_16S );
CvMat* vdisp = cvCreateMat( imageSize.height,imageSize.width, CV_8U );
CvMat* pair;
double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
CvMat _R1 = cvMat(3, 3, CV_64F, R1);
CvMat _R2 = cvMat(3, 3, CV_64F, R2);
// IF BY CALIBRATED (BOUGUET'S METHOD)
if( useUncalibrated == 0 )
{
CvMat _P1 = cvMat(3, 4, CV_64F, P1);
CvMat _P2 = cvMat(3, 4, CV_64F, P2);
cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,&_R, &_T,&_R1, &_R2, &_P1, &_P2, 0,0/*CV_CALIB_ZERO_DISPARITY*/ );
isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
//Precompute maps for cvRemap()
cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
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
cvFindFundamentalMat( &_imagePoints1,&_imagePoints2, &_F);
cvStereoRectifyUncalibrated( &_imagePoints1,&_imagePoints2, &_F,imageSize,&_H1, &_H2, 3);
cvInvert(&_M1, &_iM);
cvMatMul(&_H1, &_M1, &_R1);
cvMatMul(&_iM, &_R1, &_R1);
cvInvert(&_M2, &_iM);
cvMatMul(&_H2, &_M2, &_R2);
cvMatMul(&_iM, &_R2, &_R2);
//Precompute map for cvRemap()
cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_M1,mx1,my1);
cvInitUndistortRectifyMap(&_M2,&_D1,&_R2,&_M2,mx2,my2);
}
else
assert(0);
cvNamedWindow( "rectified", 1 );
Sleep(10000);
// RECTIFY THE IMAGES AND FIND DISPARITY MAPS
if( !isVerticalStereo )
pair = cvCreateMat( imageSize.height, imageSize.width*2,CV_8UC3 );
else
pair = cvCreateMat( imageSize.height*2, imageSize.width,CV_8UC3 );
//Setup for finding stereo correspondences
CvStereoBMState *BMState = cvCreateStereoBMState();
assert(BMState != 0);
BMState->preFilterSize=41;
BMState->preFilterCap=31;
BMState->SADWindowSize=41;
BMState->minDisparity=-64;
BMState->numberOfDisparities=128;
BMState->textureThreshold=10;
BMState->uniquenessRatio=15;
for( i = 0; i < nframes; i++ )
{
	IplImage* img1=cvQueryFrame(leftCam);
	IplImage* img2=cvQueryFrame(rightCam);
//IplImage* img1=imageList[i/2];
//IplImage* img2=imageList[(i/2)+1];
if( img1 && img2 )
{
CvMat part;
cvRemap( img1, img1r, mx1, my1 );
cvRemap( img2, img2r, mx2, my2 );
if( !isVerticalStereo || useUncalibrated != 0 )
{
// When the stereo camera is oriented vertically,
// useUncalibrated==0 does not transpose the
// image, so the epipolar lines in the rectified
// images are vertical. Stereo correspondence
// function does not support such a case.
cvFindStereoCorrespondenceBM( img1r, img2r, disp,BMState);
cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
cvNamedWindow( "disparity" );
cvShowImage( "disparity", vdisp );
}
if( !isVerticalStereo )
{
cvGetCols( pair, &part, 0, imageSize.width );
cvCvtColor( img1r, &part, CV_GRAY2BGR );
cvGetCols( pair, &part, imageSize.width,imageSize.width*2 );
cvCvtColor( img2r, &part, CV_GRAY2BGR );
for( j = 0; j < imageSize.height; j += 16 )
cvLine( pair, cvPoint(0,j),cvPoint(imageSize.width*2,j),CV_RGB(0,255,0));
}
else
{
cvGetRows( pair, &part, 0, imageSize.height );
cvCvtColor( img1r, &part, CV_GRAY2BGR );
cvGetRows( pair, &part, imageSize.height,imageSize.height*2 );
cvCvtColor( img2r, &part, CV_GRAY2BGR );
for( j = 0; j < imageSize.width; j += 16 )
cvLine( pair, cvPoint(j,0),cvPoint(j,imageSize.height*2),CV_RGB(0,255,0));
}
cvShowImage( "rectified", pair );
if( cvWaitKey() == 27 )
break;
}
cvReleaseImage( &img1 );
cvReleaseImage( &img2 );
}
cvReleaseStereoBMState(&BMState);
cvReleaseMat( &mx1 );
cvReleaseMat( &my1 );
cvReleaseMat( &mx2 );
cvReleaseMat( &my2 );
cvReleaseMat( &img1r );
cvReleaseMat( &img2r );
cvReleaseMat( &disp );
}
}
