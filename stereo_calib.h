extern void
StereoCalib(const char* imageList, int useUncalibrated);

extern int 
StereoRectify(IplImage** , IplImage** , CvMat**, CvMat**, CvMat**, CvMat** );
