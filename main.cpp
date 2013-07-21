//#include <getopt.h>             /* getopt_long() */

#include <cstring>

#include "surf.h"

#include "stereo_calib.h"

#include "capture.h"





int mainFunc(int argc,char ** argv)
{

//      para calibrar, pergunta se quer usar a calibração padrão ou quer calibrar agora
//	se for calibrar agora, pergunta se quer tornar essa calibração padrão
//	StereoCalib("../ref/stereo_calib2.txt", 0);



	
	static double tt = (double)cvGetTickCount();

	//variáveis para capture.h
	static char* dev_name = (char*) "/dev/video0";
	static io_method        io              = IO_METHOD_READ;
	static int              fd              = -1;
	struct buffer *         buffers         = NULL;
	static unsigned int     n_buffers       = 0;

	CvSize size = cvSize(320,240);
	IplImage* s_img = cvCreateImage(size,IPL_DEPTH_8U,1);
	IplImage* s_img2 = cvCreateImage(size,IPL_DEPTH_8U,1);
	int camId=0;
	
	//VARIÁVEL DE CONTROLE PARA APRESENTAÇÃO

	static const char short_options [] = "dz:hmruc";
	static const struct option
		long_options [] = {
			{ "device",     required_argument,      NULL,           'd' },
			{ "help",       no_argument,            NULL,           'h' },
			{ "mmap",       no_argument,            NULL,           'm' },
			{ "read",       no_argument,            NULL,           'r' },
			{ "userp",      no_argument,            NULL,           'u' },
			{ "calibra",	no_argument,		NULL,		'c' },
			{ "calib_my_list", required_argument,   NULL,		'z' },
			{ 0, 0, 0, 0 }
		};        

	for (;;) {
                int index;
                int c;
                
                c = getopt_long (argc, argv,
                                 short_options, long_options,
                                 &index);

                if (-1 == c)
                        break;

                switch (c) {
                case 0: /* getopt_long() flag */
                        break;

                case 'd':
                        dev_name = optarg;
                        break;

                case 'h':
                        usage (stdout, argc, argv);
                        exit (EXIT_SUCCESS);

                case 'm':
                        io = IO_METHOD_MMAP;
			break;

                case 'r':
                        io = IO_METHOD_READ;
			break;

                case 'u':
                        io = IO_METHOD_USERPTR;
			break;

		case 'c':
			StereoCalib("../ref/stereo_calib2.txt", 0);
			break;
		case 'z':
			cout << optarg << endl;
			StereoCalib(optarg,0);
			break;
			
                default:
                        usage (stderr, argc, argv);
                        exit (EXIT_FAILURE);
                }
        }
	
	open_device (dev_name,fd);

	buffers = init_device (size,dev_name,io,fd,buffers,n_buffers);

	start_capturing (io,fd,buffers,n_buffers);
	
	char c;	
	int frames = 0;

//	VARIÁVEIS DE CONTROLE
	bool rectify=false;
	bool surfing=false;
	bool smooth=false;
	bool equalize=false;	
	bool apresenta=false;

//	VARIÁVEIS PARA RETIFICAÇÃO DA IMAGEM
	IplImage* rect = cvCreateImage(size,IPL_DEPTH_8U,1);
	IplImage* rect2 = cvCreateImage(size,IPL_DEPTH_8U,1);
        CvMat* mx1 = cvCreateMat( size.height,
                        size.width, CV_32F );
        CvMat* my1 = cvCreateMat( size.height,
                        size.width, CV_32F );
        CvMat* mx2 = cvCreateMat( size.height,
                        size.width, CV_32F );
        CvMat* my2 = cvCreateMat( size.height,
                        size.width, CV_32F );

	for(;;)
	{
//		cout << "aqui" << endl;
		getIplImage ( &s_img, &s_img2, io, fd, buffers, n_buffers, camId );
//		cout << "aqui1" << endl;
		
		if (!apresenta)
		{
			cvShowImage("Main",s_img);
			cvShowImage("Main2",s_img2);
		}
                c = cvWaitKey(0);
                if ( c == 27 )
                        exit(0);
		if ( c == 'a' )
		{
			if(apresenta==false) apresenta=true;
			else apresenta = false;
		}
		if ( c == 'c' )
		{
			grava_imagem_calibragem( s_img, s_img2, frames );
			frames++;
		}
		if ( c == 'g' )
		{
			if(!smooth) smooth = true;
			else
			{
				smooth = false;
				cvDestroyWindow( "SMOOTH");
				cvDestroyWindow( "SMOOTH2");
			}
		}
		if ( c == 'e' )
		{
			if(!equalize) equalize = true;
			else 
			{
				equalize = false;
				cvDestroyWindow( "EQUALIZE");
				cvDestroyWindow( "EQUALIZE2");
			}
		} 
		if ( c == 's' )
		{
			if(!surfing) surfing = true;
			else surfing = false;
		}
		if ( c == 'r' )
		{
			if(!rectify)
			{
				rectify = true;
				if ( StereoRectify( &s_img, &s_img2, &mx1, &my1, &mx2, &my2) != 0 ) 
					{ cout << "erro em StereoRectify()" << endl;break; }
			}
			else rectify = false;
		}
		if ( c == 't' )
		{
//			IplImage* tsu = cvLoadImage( "im0.ppm", CV_LOAD_IMAGE_GRAYSCALE); //venus
//			IplImage* tsu2 = cvLoadImage( "im4.ppm", CV_LOAD_IMAGE_GRAYSCALE); //venus
//			IplImage* tsu = cvLoadImage( "mim0.pgm", CV_LOAD_IMAGE_GRAYSCALE);
//			IplImage* tsu2 = cvLoadImage( "mim1.pgm", CV_LOAD_IMAGE_GRAYSCALE);
//			IplImage* tsu = cvLoadImage( "cim2.ppm", CV_LOAD_IMAGE_GRAYSCALE);
//			IplImage* tsu2 = cvLoadImage( "cim3.ppm", CV_LOAD_IMAGE_GRAYSCALE);
//			IplImage* tsu = cvLoadImage( "tim2.ppm", CV_LOAD_IMAGE_GRAYSCALE);
//			IplImage* tsu2 = cvLoadImage( "tim3.ppm", CV_LOAD_IMAGE_GRAYSCALE);
//			IplImage* tsu = cvLoadImage( "im2.ppm", CV_LOAD_IMAGE_GRAYSCALE); //poster
//			IplImage* tsu2 = cvLoadImage( "im5.ppm", CV_LOAD_IMAGE_GRAYSCALE); //poster
			IplImage* tsu = cvLoadImage("scene1.row3.col1.ppm", CV_LOAD_IMAGE_GRAYSCALE); //tsukuba
			IplImage* tsu2 = cvLoadImage("scene1.row3.col2.ppm", CV_LOAD_IMAGE_GRAYSCALE); //tsukuba
			surf_estereo(tsu2, tsu,apresenta);
		}
		///processando
		if (smooth)
		{
//			cvSmooth(s_img,rect,CV_MEDIAN,3,3,0);
//			cvSmooth(s_img2,rect2,CV_MEDIAN,3,3,0);
			cvSmooth(s_img,rect,CV_GAUSSIAN,3,3,0);
			cvSmooth(s_img2,rect2,CV_GAUSSIAN,3,3,0);
			cvCopy(rect,s_img);
			cvCopy(rect2,s_img2);
			if(!apresenta)
			{
				cvShowImage("SMOOTH",s_img);
				cvShowImage("SMOOTH2",s_img2);
			}
		}
		if (equalize)
		{
			cvEqualizeHist(s_img,s_img);
			cvEqualizeHist(s_img2,s_img2);
			if(!apresenta)
			{
				cvShowImage("EQUALIZE",s_img);
				cvShowImage("EQUALIZE2",s_img2);
			}
		}
		if (rectify) 
		{
			cvRemap( s_img, rect, mx1, my1 );
			cvRemap( s_img2, rect2, mx2, my2 );
			cvCopy(rect,s_img);
			cvCopy(rect2,s_img2);
		}
		if (surfing) surf_estereo(s_img,s_img2,apresenta);

	}

        stop_capturing (io,fd);

        uninit_device (io,buffers,n_buffers);

        close_device (fd);

        exit (EXIT_SUCCESS);

        return 0;
}
