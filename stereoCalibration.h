#include "cv.h"
#include "cxmisc.h"
#include "highgui.h"
#include "cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>
using namespace cv;



void stereoCalib(int xPatSize,int yPatSize,int n,CvMat* IMLeft,CvMat* IMRight,CvMat* FMat,CvMat* dMatL,CvMat* dMatR);

void rectifyImage(Mat left,Mat right);