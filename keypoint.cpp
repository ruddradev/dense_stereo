/*#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
*/
#include <cvaux.h>
#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <ml.h>
#include <vector>
#include <string>
#include "stereo1.h"
using namespace cv;
using namespace std;
vector<KeyPoint> getKeypoints(Mat Image, char* imageName)
{
	vector<KeyPoint> keypoints;
	Ptr<FeatureDetector> featureDetector= FeatureDetector::create("SURF");

	featureDetector ->detect(Image, keypoints);

	Ptr<DescriptorExtractor> featureExtractor=DescriptorExtractor::create("SURF");
	Mat descriptors;
	featureExtractor->compute(Image, keypoints, descriptors);
	Mat outputImage;
	Scalar keypointColor=Scalar (255,0,0);
	drawKeypoints(Image,keypoints,outputImage,keypointColor,DrawMatchesFlags::DEFAULT);

	//namedWindow(imageName);
	//imshow(imageName,outputImage);
	char c=' ';
	//while(c=waitKey(0)!='q');
	return keypoints;
}
