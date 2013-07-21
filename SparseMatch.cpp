#include <cvaux.h>
#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <ml.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include "keypoint.h"
#include "stereo1.h"
#include "highgui.h"
//#include "SparseMatch.h"
using namespace std;
using namespace cv;
vector<KeyPoint> imageKeypoints[2];
vector<Point2f> leftPts,rightPts;
vector<DMatch> matchPoints(double matchThresh=2)
{
	vector<DMatch> matches;
	Mat left=Mat(imagePair[0]);
	Mat right=Mat(imagePair[1]);
	Mat leftOp;
	Mat rightOp;
	Mat matchImage;
	vector<char> mask;
	//namedWindow("rightImage");
	//imshow("rightImage",right);
	imageKeypoints[0]=getKeypoints(left,"left");
	imageKeypoints[1]=getKeypoints(right,"right");
	Scalar keypointColor=Scalar (255,0,0);
	//drawKeypoints(left,imageKeypoints[0],leftOp,keypointColor,DrawMatchesFlags::DEFAULT);
	//drawKeypoints(right,imageKeypoints[1],rightOp,keypointColor,DrawMatchesFlags::DEFAULT);
	//imwrite("C:\\mimg\\keypointsleft.jpg",leftOp);
	//imwrite("C:\\mimg\\keypointsright.jpg",rightOp);
	SurfDescriptorExtractor extractor;

	Mat descriptors_1, descriptors_2;

  extractor.compute( left, imageKeypoints[0], descriptors_1 );
  extractor.compute( right, imageKeypoints[1], descriptors_2 );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  //std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );
fflush(stdout);
  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
  //-- PS.- radiusMatch can also be used here.
  vector< DMatch > good_matches;
  //implementing adaptive threshold binary search for more than 8 pts
  while(true)
  {
double initThresh=matchThresh;  
  for( int i = 0; i < descriptors_1.rows; i++ )
  { 
	  initThresh=matchThresh;
	  if( matches[i].distance < matchThresh*min_dist )
    { good_matches.push_back( matches[i]); }
  }
  cout<<good_matches.size()<<endl;
if(good_matches.size()<20)
	matchThresh= initThresh+ (20+(good_matches.size()))*0.04;
else if(good_matches.size()>40)
matchThresh= initThresh- ((good_matches.size())-40)*0.08;
else
break;
}
  //-- Draw only "good" matches
  Mat img_matches;
	drawMatches(left,imageKeypoints[0],right,imageKeypoints[1],good_matches,img_matches,Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	imwrite("C:\\mimg\\matchImage.jpg",img_matches);
	//cvWaitKey();
	return good_matches;
}
Mat findCorrespondences()			//returns 2 matrices the query image matches i.e left &  the training img right
{

	vector<DMatch> matches;
	Mat corr;
	Point2f leftPt,rightPt;
	//vector<double> disparity;
	//initCameras();
	changeImagePair();
	matches=matchPoints();
	corr=Mat(matches.size(),3,CV_64F);
	ofstream matchData;
	float disp;
	//stringstream ss (stringstream::in | stringstream::out);
	matchData.open("C:\\mimg\\data.txt"); 
	//Point3f dispPoint;
	for(size_t i = 0; i < matches.size(); i++)
{
    leftPt = imageKeypoints[0][matches[i].queryIdx].pt;
    rightPt = imageKeypoints[1][matches[i].trainIdx].pt;
	leftPts.push_back(leftPt);
	rightPts.push_back(rightPt); 
	disp= abs(sqrt(pow((leftPt.x- rightPt.x),2)+pow((leftPt.y-rightPt.y),2)));
	//disparity.push_back((float)disp); 
	//int n = corr.channels();
	corr.at<double>(i,0)=leftPt.x;
	corr.at<double>(i,1)=leftPt.y;
	corr.at<double>(i,2)=disp;
	/*double* ptr = (double*)corr.data + corr.step*i;
	// sample point in image
	ptr[0]=leftPt.x;
	ptr[1]=leftPt.y;
	ptr[2]=disp;
	ptr[3]=1;*/
	//matchData<<leftPt.x<<" "<<leftPt.y<<" "<<disp<<endl; 
	cout<<i<<endl;
	//Beep(600,400);
	//corr.push_back(dispPoint);
	}
	matchData.close(); 
	Beep(400,400);
	//corr.push_back(rightPts); 	
	return corr;
}
Mat makePointCloud()
{
	Mat coordinates;
	Mat fundamental= Mat(3,3,CV_64F);
	Mat dispPoints=findCorrespondences();
	fundamental=findFundamentalMat(leftPts,rightPts);
	Beep(400,500);
	coordinates=Mat(dispPoints.size().height,4,CV_64F);
	CvMat* pLeft=(CvMat*)cvLoad("D:\\Camera_calib_data\\P1.xml");
	CvMat* pRight=(CvMat*)cvLoad("D:\\Camera_calib_data\\P2.xml");
	CvMat* Ml=(CvMat*)cvLoad("D:\\Camera_calib_data\\M1.xml");
	CvMat* reproj=(CvMat*)cvLoad("D:\\Camera_calib_data\\M1.xml");
	
	Mat Mleft=Mat(Ml);
	CvMat* Mr=(CvMat*)cvLoad("D:\\Camera_calib_data\\M2.xml");
	Mat Mright=Mat(Mr);
	Mat pL=Mat(pLeft);
	Mat pR=Mat(pRight);
	Mat dist= Mat(3,2,CV_32FC3);
	Mat reprojection(reproj);
	Mat EMat=Mleft.t().mul(fundamental).mul(Mright);
	cvSave("D:\\Camera_calib_data\\FCalc.xml",&(CvMat)fundamental);
	Beep(1000,600);
	cvSave("D:\\Camera_calib_data\\ECalc.xml",&(CvMat)EMat);
	//Mat rot= SVD::
	
	//(dispPoints,coordinates ,reproj);	
	coordinates= reprojection.mul(dispPoints.t());
	Beep(300,1000);
	ofstream coord;
	coord.open("C:\\mimg\\coord.txt");
	Mat pointCoords= coordinates.t();
	for(size_t i=0;i<=dispPoints.size().height;i++)
	{
		double* ptr = (double*)pointCoords.data + pointCoords.step*i;
		coord<<" "<<ptr[0]<<" "<<ptr[1]<<" "<<ptr[2]<<" "<<ptr[3]<<endl;
	}
	coord.close(); 
	return coordinates;
}