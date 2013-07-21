extern IplImage* imagePair[2];
void initCameras();
void changeImagePair();
void convertQMatrix();
void StereoCalib(const char* imageList, int nx, int ny,int useUncalibrated, float _squareSize,int isCalibrated);