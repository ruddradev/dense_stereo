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

void makeList(vector<vector<IplImage*>> &imageList,int n);

void StereoCalib(/*const char* imageList,*/ int n_frames,int nx, int ny, int useUncalibrated);

