
typedef enum {
	IO_METHOD_READ,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR,
} io_method;

struct buffer {
        void *                  start;
        size_t                  length;
};

extern void
errno_exit                      (const char *           );

extern int
xioctl                          (int                    ,
                                 int                    ,
                                 void *                 );

extern void
grava_imagem(IplImage * , int& );

extern void
grava_imagem_calibragem(IplImage* , IplImage*, int );

extern IplImage*
carrega_imagem_gravada(int& , CvSize );

extern IplImage*
carrega_imagem_gravada_calib(int& , CvSize );

extern int 
read_frame( IplImage ** , io_method , int , buffer * , unsigned int );

extern void  
getIplImage(IplImage ** , IplImage ** , io_method , int , buffer * , unsigned int , int& );

extern void
stop_capturing(io_method , int );

extern void
start_capturing(io_method , int , buffer * , unsigned int );

extern void
uninit_device(io_method , buffer * , unsigned int );

extern buffer *
init_read(unsigned int , buffer * );

extern void
init_mmap(char* , int , buffer * , unsigned int );

extern void
init_userp(unsigned int , char* , int , buffer * , unsigned int );

extern buffer * 
init_device(CvSize , char* , io_method , int , buffer * , unsigned int );

extern void
close_device(int& );

extern void
open_device(char* , int& );

extern void
usage(FILE * , int ,char ** );
