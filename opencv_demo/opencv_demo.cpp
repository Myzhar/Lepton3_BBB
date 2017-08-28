#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <string>
#include <chrono>
#include <thread>
#ifndef WRITE_JPEG
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#endif
#include "lepton3_grabber.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

static bool do_exit = false;
#ifndef WRITE_JPEG
int fbfd = 0;
struct fb_fix_screeninfo finfo;
long int screensize = 0;
char *fbp = 0;
#endif


void exit_handler(int s)
{
	if(s==2)
	{
		cout << endl << "Ctrl+C pressed..." << endl;		
		do_exit = true;
	} 
}

int main (int argc, char *argv[])
{
	cout << "OpenCV demo for Lepton3 on BeagleBoard Blue" << endl;
	
	// >>>>> Enable Ctrl+C
	struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    // <<<<< Enable Ctrl+C
	
	L3_grabber grabber;
	
	grabber.start();
	
	uint64_t frameIdx=0;
	char image_name[32];
	
#ifndef WRITE_JPEG		
	fbfd = open("/dev/fb0", O_RDWR);

	if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo)) {
		printf("Error reading screen information.\n");
		exit(-1);
	}

	// map fb to user mem 
	screensize = finfo.smem_len;
	fbp = (char*)mmap(0, 
		screensize, 
		PROT_READ | PROT_WRITE, 
		MAP_SHARED, 
		fbfd, 
		0);

	if ((int)fbp == -1) {
		printf("Failed to mmap.\n");
	}
#endif

	while(!do_exit)
	{
		int w;
		int h;
		
		const char* data = grabber.getLastFrame( &w, &h );

		if( data && w!=-1 && h!=-1 )
		{
#ifdef WRITE_JPEG		
			sprintf(image_name, "IMG_%.6d.jpg", frameIdx);
			string imgStr = image_name;
			
			cv::Mat frame( h,w, CV_8UC1 );
			memcpy( frame.data, data, w*h );
			
			cv::imwrite( image_name, frame );
			
			cout << "Saved: " << image_name << "[" << w << "x" << h << "]" << endl;
			
			frameIdx++;
#else
			cv::Mat frame( h,w, CV_8UC1 );
			memcpy( frame.data, data, w*h );
#endif
		}
		
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	
	grabber.stop();

#ifndef WRITE_JPEG
	munmap(fbp, screensize);
	close(fbfd);
#endif

    return EXIT_SUCCESS;
}
