//includes para SURF
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <iostream>
#include <vector>

#include "surf.h"

using namespace std;
void help()
{
        printf(
                        "This program demonstrated the use of the SURF Detector and Descriptor using\n"
                        "either FLANN (fast approx nearst neighbor classification) or brute force matching\n"
                        "on planar objects.\n"
                        "Call:\n"
                        "./find_obj [<object_filename default box.png> <scene_filename default box_in_scene.png>]\n\n"
                        );

}

double
compareSURFDescriptors( const float* d1, const float* d2, double best, int length )
{
    double total_cost = 0;
    assert( length % 4 == 0 );
    for( int i = 0; i < length; i += 4 )
    {
        double t0 = d1[i] - d2[i];
        double t1 = d1[i+1] - d2[i+1];
        double t2 = d1[i+2] - d2[i+2];
        double t3 = d1[i+3] - d2[i+3];
        total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
        if( total_cost > best )
            break;
    }
    return total_cost;
}

int
naiveNearestNeighbor( const float* vec, int laplacian,
                      const CvSeq* model_keypoints,
                      const CvSeq* model_descriptors )
{
    int length = (int)(model_descriptors->elem_size/sizeof(float));
    int i, neighbor = -1;
    double d, dist1 = 1e6, dist2 = 1e6;
    CvSeqReader reader, kreader;
    cvStartReadSeq( model_keypoints, &kreader, 0 );
    cvStartReadSeq( model_descriptors, &reader, 0 );

    for( i = 0; i < model_descriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* mvec = (const float*)reader.ptr;
        CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
        if( laplacian != kp->laplacian )
            continue;
        d = compareSURFDescriptors( vec, mvec, dist2, length );
        if( d < dist1 )
        {
            dist2 = dist1;
            dist1 = d;
            neighbor = i;
        }
        else if ( d < dist2 )
            dist2 = d;
    }
    if ( dist1 < 0.6*dist2 )
        return neighbor;
    return -1;
}

void
findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
           const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
    int i;
    CvSeqReader reader, kreader;
    cvStartReadSeq( objectKeypoints, &kreader );
    cvStartReadSeq( objectDescriptors, &reader );
    ptpairs.clear();

    for( i = 0; i < objectDescriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* descriptor = (const float*)reader.ptr;
        CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
        int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
        if( nearest_neighbor >= 0 )
        {
            ptpairs.push_back(i);
            ptpairs.push_back(nearest_neighbor);
        }
    }
}

void
flannFindPairs( const CvSeq*, const CvSeq* objectDescriptors,
           const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
        int length = (int)(objectDescriptors->elem_size/sizeof(float));

    cv::Mat m_object(objectDescriptors->total, length, CV_32F);
        cv::Mat m_image(imageDescriptors->total, length, CV_32F);


        // copy descriptors
    CvSeqReader obj_reader;
        float* obj_ptr = m_object.ptr<float>(0);
    cvStartReadSeq( objectDescriptors, &obj_reader );
    for(int i = 0; i < objectDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)obj_reader.ptr;
        CV_NEXT_SEQ_ELEM( obj_reader.seq->elem_size, obj_reader );
        memcpy(obj_ptr, descriptor, length*sizeof(float));
        obj_ptr += length;
    }
    CvSeqReader img_reader;
        float* img_ptr = m_image.ptr<float>(0);
    cvStartReadSeq( imageDescriptors, &img_reader );
    for(int i = 0; i < imageDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)img_reader.ptr;
        CV_NEXT_SEQ_ELEM( img_reader.seq->elem_size, img_reader );
        memcpy(img_ptr, descriptor, length*sizeof(float));
        img_ptr += length;
    }

    // find nearest neighbors using FLANN
    cv::Mat m_indices(objectDescriptors->total, 2, CV_32S);
    cv::Mat m_dists(objectDescriptors->total, 2, CV_32F);
    cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(4));  // using 4 randomized kdtrees
    flann_index.knnSearch(m_object, m_indices, m_dists, 2, cv::flann::SearchParams(64) ); // maximum number of leafs checked

    int* indices_ptr = m_indices.ptr<int>(0);
    float* dists_ptr = m_dists.ptr<float>(0);
    for (int i=0;i<m_indices.rows;++i) {
        if (dists_ptr[2*i]<0.6*dists_ptr[2*i+1]) {
                ptpairs.push_back(i);
                ptpairs.push_back(indices_ptr[2*i]);
        }
    }
}


/* a rough implementation for object location */
int
locatePlanarObject( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
                    const CvSeq* imageKeypoints, const CvSeq* imageDescriptors,
                    const CvPoint src_corners[4], CvPoint dst_corners[4] )
{
    double h[9];
    CvMat _h = cvMat(3, 3, CV_64F, h);
    vector<int> ptpairs;
    vector<CvPoint2D32f> pt1, pt2;
    CvMat _pt1, _pt2;
    int i, n;

#ifdef USE_FLANN
    flannFindPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
#else
    findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
#endif

    n = (int)(ptpairs.size()/2);
    if( n < 4 )
        return 0;

    pt1.resize(n);
    pt2.resize(n);
    for( i = 0; i < n; i++ )
    {
        pt1[i] = ((CvSURFPoint*)cvGetSeqElem(objectKeypoints,ptpairs[i*2]))->pt;
        pt2[i] = ((CvSURFPoint*)cvGetSeqElem(imageKeypoints,ptpairs[i*2+1]))->pt;
    }

    _pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
    _pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );
    if( !cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, 5 ))
        return 0;

    for( i = 0; i < 4; i++ )
    {
        double x = src_corners[i].x, y = src_corners[i].y;
        double Z = 1./(h[6]*x + h[7]*y + h[8]);
        double X = (h[0]*x + h[1]*y + h[2])*Z;
        double Y = (h[3]*x + h[4]*y + h[5])*Z;
        dst_corners[i] = cvPoint(cvRound(X), cvRound(Y));
    }

    return 1;
}

void teste_SURF()
{
	const char* object_filename = "../data/box.png";
	const char* scene_filename = "../data/box_in_scene.png";

	CvMemStorage* storage = cvCreateMemStorage(0);
	help();
	cvNamedWindow("Object", 1);
	cvNamedWindow("Object Correspond", 1);

        static CvScalar colors[] =
        {
                {{0,0,255}},
                {{0,128,255}},
                {{0,255,255}},
                {{0,255,0}},
                {{255,128,0}},
                {{255,255,0}},
                {{255,0,0}},
                {{255,0,255}},
                {{255,255,255}}
        };

	IplImage* object = cvLoadImage( object_filename, CV_LOAD_IMAGE_GRAYSCALE );
	IplImage* image = cvLoadImage( scene_filename, CV_LOAD_IMAGE_GRAYSCALE );
	if( !object || !image )
	{
		fprintf( stderr, "Can not load %s and/or %s\n"
				"Usage: find_obj [<object_filename> <scene_filename>]\n",
				object_filename, scene_filename );
		exit(-1);
	}

	IplImage* object_color = cvCreateImage(cvGetSize(object), 8, 3);
	cvCvtColor( object, object_color, CV_GRAY2BGR );

	CvSeq *objectKeypoints = 0, *objectDescriptors = 0;
	CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
	int i;
	CvSURFParams params = cvSURFParams(500, 1);

	double tty = (double)cvGetTickCount();
	cvExtractSURF( object, 0, &objectKeypoints, &objectDescriptors, storage, params );
	printf("Object Descriptors: %d\n", objectDescriptors->total);
	cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );
	printf("Image Descriptors: %d\n", imageDescriptors->total);
	tty = (double)cvGetTickCount() - tty;
	printf( "Extraction time = %gms\n", tty/(cvGetTickFrequency()*1000.));
	CvPoint src_corners[4] = {{0,0}, {object->width,0}, {object->width, object->height}, {0, object->height}};
	CvPoint dst_corners[4];
	IplImage* correspond = cvCreateImage( cvSize(image->width, object->height+image->height), 8, 1 );
	cvSetImageROI( correspond, cvRect( 0, 0, object->width, object->height ) );
	cvCopy( object, correspond );
	cvSetImageROI( correspond, cvRect( 0, object->height, correspond->width, correspond->height ) );
	cvCopy( image, correspond );
	cvResetImageROI( correspond );

#ifdef USE_FLANN
	printf("Using approximate nearest neighbor search\n");
#endif

	if( locatePlanarObject( objectKeypoints, objectDescriptors, imageKeypoints,
				imageDescriptors, src_corners, dst_corners ))
	{
		for( i = 0; i < 4; i++ )
		{
			CvPoint r1 = dst_corners[i%4];
			CvPoint r2 = dst_corners[(i+1)%4];
			cvLine( correspond, cvPoint(r1.x, r1.y+object->height ),
					cvPoint(r2.x, r2.y+object->height ), colors[8] );
		}
	}

	vector<int> ptpairs;
#ifdef USE_FLANN
	flannFindPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
#else
	findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
#endif
	for( i = 0; i < (int)ptpairs.size(); i += 2 )
	{
		CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( objectKeypoints, ptpairs[i] );
		CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, ptpairs[i+1] );
		cvLine( correspond, cvPointFrom32f(r1->pt),
				cvPoint(cvRound(r2->pt.x), cvRound(r2->pt.y+object->height)), colors[8] );
	}

	cvShowImage( "Object Correspond", correspond );
	for( i = 0; i < objectKeypoints->total; i++ )
	{
		CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( objectKeypoints, i );
		CvPoint center;
		int radius;
		center.x = cvRound(r->pt.x);
		center.y = cvRound(r->pt.y);
		radius = cvRound(r->size*1.2/9.*2);
		cvCircle( object_color, center, radius, colors[0], 1, 8, 0 );
	}
	cvShowImage( "Object", object_color );

	cvWaitKey(0);

	cvDestroyWindow("Object");
	cvDestroyWindow("Object SURF");
	cvDestroyWindow("Object Correspond");
}




void surf_estereo(IplImage* cinza, IplImage* cinza1, bool apresenta)
{

	CvMemStorage* storage = cvCreateMemStorage(0);

	//                      cvNamedWindow("Object", 1);
	//                      cvNamedWindow("Object Correspond", 1);

//        static CvScalar colors[] =
//        {
//                {{0,0,0}},
//                {{8,8,8}},
//                {{16,16,16}},
//                {{24,24,24}},
//                {{32,32,32}},
//                {{40,40,40}},
//                {{48,48,48}},
//                {{56,56,56}},
//                {{64,64,64}},
//                {{72,72,72}},
//                {{80,80,80}},
//                {{88,88,88}},
//                {{96,96,96}},
//                {{104,104,104}},
//                {{112,112,112}},
//                {{120,120,120}},
//                {{128,128,128}},
//                {{136,136,136}},
//                {{144,144,144}},
//                {{152,152,152}},
//                {{160,160,160}},
//                {{168,168,168}},
//                {{176,176,176}},
//                {{184,184,184}},
//                {{192,192,192}},
//                {{200,200,200}},
//                {{208,208,208}},
//                {{216,216,216}},
//                {{224,224,224}},
//                {{232,232,232}},
//                {{240,240,240}},
//                {{248,248,248}},
//                {{255,255,255}}
//        };
        static CvScalar colors[] =
        {
                {{0,0,0}},
                {{0,8,0}},
                {{0,16,0}},
                {{0,24,0}},
                {{0,32,0}},
                {{0,40,0}},
                {{0,48,0}},
                {{0,56,0}},
                {{0,64,0}},
                {{0,72,0}},
                {{0,80,0}},
                {{0,88,0}},
                {{0,96,0}},
                {{0,104,0}},
                {{0,112,0}},
                {{0,120,0}},
                {{0,128,0}},
                {{0,136,0}},
                {{0,144,0}},
                {{0,152,0}},
                {{0,160,0}},
                {{0,168,0}},
                {{0,176,0}},
                {{0,184,0}},
                {{0,192,0}},
                {{0,200,0}},
                {{0,208,0}},
                {{0,216,0}},
                {{0,224,0}},
                {{0,232,0}},
                {{0,240,0}},
                {{0,248,0}},
                {{0,255,0}}
        };


	//                      IplImage* object = cinza;
	//                      IplImage* image = cinza1;

//	IplImage* object_color = cvCreateImage(cvGetSize(object), 8, 3);
//	cvCvtColor( cinza, object_color, CV_GRAY2BGR );
        CvSeq *objectKeypoints = 0, *objectDescriptors = 0;
        CvSeq *imageKeypoints = 0, *imageDescriptors = 0;

	int i;
        CvSURFParams params = cvSURFParams(500, 1);
	cvExtractSURF( cinza, 0, &objectKeypoints, &objectDescriptors, storage, params );
	cvExtractSURF( cinza1, 0, &imageKeypoints, &imageDescriptors, storage, params );
//	char txCore[64];
//	sprintf( txCore, "%d",objectKeypoints->total);
//	CvPoint text_origin;
//	text_origin.x = 10;
//	text_origin.y = 20;
//	CvFont font = cvFont( 1, 1 );
//	cvPutText(object_color, txCore, text_origin, &font, CV_RGB(0,255,0));
	CvPoint src_corners[4] = {{0,0}, {cinza->width,0}, {cinza->width, cinza->height}, {0, cinza->height}};
	CvPoint dst_corners[4];
	IplImage* correspond = cvCreateImage( cvSize(cinza1->width, cinza->height+cinza1->height), 8, 1 );
	cvSetImageROI( correspond, cvRect( 0, 0, cinza->width, cinza->height ) );
	cvCopy( cinza, correspond );
	cvSetImageROI( correspond, cvRect( 0, cinza->height, correspond->width, correspond->height ) );
	cvCopy( cinza1, correspond );
	cvResetImageROI( correspond );
	//              
	//#ifdef USE_FLANN
	//      printf("Using approximate nearest neighbor search\n");
	//#endif

	//if( locatePlanarObject( objectKeypoints, objectDescriptors, imageKeypoints,
	//                              imageDescriptors, src_corners, dst_corners ))
	//                      {
	//                              for( i = 0; i < 4; i++ )
	//                              {
	//                                      CvPoint r1 = dst_corners[i%4];
	//                                      CvPoint r2 = dst_corners[(i+1)%4];
	//                                      cvLine( correspond, cvPoint(r1.x, r1.y+cinza->height ),
	//                                      cvPoint(r2.x, r2.y+cinza->height ), colors[8] );
	//                              }
	//                      }
	vector<int> ptpairs;
	//
#ifdef USE_FLANN
	flannFindPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
#else
	findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
#endif
	IplImage* temp = cvCreateImage( cvSize(cinza1->width, cinza->height+cinza1->height), 8, 3 );
	cvCvtColor(correspond, temp,  CV_GRAY2BGR );

	for( i = 0; i < (int)ptpairs.size(); i += 2 )
	{
		CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( objectKeypoints, ptpairs[i] );
		CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, ptpairs[i+1] );
		cvLine( temp, cvPointFrom32f(r1->pt),
				cvPoint(cvRound(r2->pt.x), cvRound(r2->pt.y+cinza->height)), CV_RGB(0,255,0) );
	}
	IplImage* cloud = cvCreateImage(cvGetSize(cinza),IPL_DEPTH_8U,3);
//	cvSetZero(cloud); // pinta o fundo de preto
	cvSet(cloud,CV_RGB(128,0,0)); // pinta o fundo de vermelho

	int contA1=0;
	int contA2=0;
	int contA3=0;

	for( i = 0; i < (int)ptpairs.size(); i += 2 )
	{
		CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( objectKeypoints, ptpairs[i] );
		CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, ptpairs[i+1] );
		float dif;
		dif = r2->pt.y - r1->pt.y;
		if ((dif*dif)<9)
		{
			dif = r2->pt.x - r1->pt.x;
//			cout << "dif[" << i << "] = " << dif << endl;
			// DEIXA O PONTO A SER PLOTADO NO MAPA DE DISPARIDADE NO MEIO ENTRE ESQUERDA E DIREITA
			r1->pt.x += dif/2;
			if(dif>-18 && dif <= -10)
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[1], 2 );
			if(dif>-10 && dif <= -9 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[2], 2 );
			if(dif>-9 && dif <= -8 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[3], 2 );
			if(dif>-8 && dif <= -7 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[4], 2 );
			if(dif>-7 && dif <= -6 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[5], 2 );
			if(dif>-6 && dif <= -5 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[6], 2 );
			if(dif>-5 && dif <= -4 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[7], 2 );
			if(dif>-4 && dif <= -3 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[8], 2 );
			if(dif>-3 && dif <= -2 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[9], 2 );
			if(dif>-2 && dif <= -1 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[10], 2 );
			if(dif>-1 && dif <= 0 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[11], 2 );
			if(dif> 0 && dif <= 1 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[12], 2 );
			if(dif> 1 && dif <= 2 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[13], 2 );
			if(dif> 2 && dif <= 3 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[14], 2 );
			if(dif> 3 && dif <= 4 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[15], 2 );
			if(dif> 4 && dif <= 5 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[16], 2 );
			if(dif> 5 && dif <= 6 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[17], 2 );
			if(dif> 6 && dif <= 7 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[18], 2 );
			if(dif> 7 && dif <= 8 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[19], 2 );
			if(dif> 8 && dif <= 9 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[20], 2 );
			if(dif> 9 && dif <= 10 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[21], 2 );
			if(dif> 10 && dif <= 11 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[22], 2 );
			if(dif> 11 && dif <= 12 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[23], 2 );
			if(dif> 12 && dif <= 13 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[24], 2 );
			if(dif> 13 && dif <= 14 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[25], 2 );
			if(dif> 14 && dif <= 15 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[26], 2 );
			if(dif> 15 && dif <= 17 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[27], 2 );
			if(dif> 17 && dif <= 20 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[28], 2 );
			if(dif> 20 && dif <= 24 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[29], 2 );
			if(dif> 24 && dif <= 29 )
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[30], 2 );
			if(dif> 29 && dif <= 50 )
			{
				cvCircle( cloud, cvPointFrom32f(r1->pt), 1, colors[31], 2 );
				if (apresenta)
				{
					if(r1->pt.x > 10 && r1->pt.x < 110)
						contA2++;
					if(r1->pt.x > 110 && r1->pt.x < 210)
						contA1++;
					if(r1->pt.x > 210 && r1->pt.x < 310)
						contA3++;
				}
			}
		}
	}
//	cvSaveImage( "tsukuba_correspond.png",temp );
	if(!apresenta) cvShowImage( "Object Correspond", temp );
	if(apresenta)
	{
		if (contA1 > 8 )
			cout << "ALARME 1 " << endl;
		if (contA2 > 8 )
			cout << "ALARME 2 " << endl;
		if (contA3 > 8 )
			cout << "ALARME 3 " << endl;
	}
	cvShowImage( "cloud", cloud);
/*	for( i = 0; i < objectKeypoints->total; i++ )
	{
		CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( objectKeypoints, i );
		CvPoint center;
		int radius;
		center.x = cvRound(r->pt.x);
		center.y = cvRound(r->pt.y);
		radius = cvRound(r->size*1.2/9.*2);
		cvCircle( object_color, center, radius, colors[0], 1, 8, 0 );
	}*/
//	cvShowImage( "Object", cinza );
	//flagsurf=0;
	cvReleaseImage(&correspond);
	cvReleaseImage(&temp);
	cvReleaseImage(&cloud);
//	char c = cvWaitKey(10);
//	if ( c == 27 )
//		exit(0);

	cvReleaseMemStorage(&storage);

//	(&objectKeypoints);
//	(&objectDescriptors);
//	(&imageKeypoints);
//	(&imageDescriptors);
//	(&params);
	


}


