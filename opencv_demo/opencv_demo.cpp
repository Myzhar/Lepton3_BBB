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
#ifndef WRITE_JPEG
#include <opencv2/imgproc/imgproc.hpp>
#endif

using namespace std;

static bool do_exit = false;
#ifndef WRITE_JPEG
int fbfd = 0;
struct fb_fix_screeninfo finfo;
long int screensize = 0;
uint16_t *fbp = 0;

#define RGB565(r, g, b) ((((r)&0xf8)<<8) | (((g)&0xfc)<<3) | ((b)>>3))

uint16_t iron_palette[128] = {
	RGB565(0, 0, 0 ),
	RGB565(0, 0, 36 ),
	RGB565(0, 0, 36 ),
	RGB565(0, 0, 36 ),
	RGB565(0, 0, 36 ),
	RGB565(0, 0, 36 ),
	RGB565(0, 0, 36 ),
	RGB565(0, 0, 36 ),
	RGB565(0, 0, 36 ),
	RGB565(0, 0, 36 ),
	RGB565(0, 0, 51 ),
	RGB565(0, 0, 66 ),
	RGB565(0, 0, 81 ),
	RGB565(2, 0, 90 ),
	RGB565(4, 0, 99 ),
	RGB565(7, 0, 106 ),
	RGB565(11, 0, 115 ),
	RGB565(14, 0, 119 ),
	RGB565(20, 0, 123 ),
	RGB565(27, 0, 128 ),
	RGB565(33, 0, 133 ),
	RGB565(41, 0, 137 ),
	RGB565(48, 0, 140 ),
	RGB565(55, 0, 143 ),
	RGB565(61, 0, 146 ),
	RGB565(66, 0, 149 ),
	RGB565(72, 0, 150 ),
	RGB565(78, 0, 151 ),
	RGB565(84, 0, 152 ),
	RGB565(91, 0, 153 ),
	RGB565(97, 0, 155 ),
	RGB565(104, 0, 155 ),
	RGB565(110, 0, 156 ),
	RGB565(115, 0, 157 ),
	RGB565(122, 0, 157 ),
	RGB565(128, 0, 157 ),
	RGB565(134, 0, 157 ),
	RGB565(139, 0, 157 ),
	RGB565(146, 0, 156 ),
	RGB565(152, 0, 155 ),
	RGB565(157, 0, 155 ),
	RGB565(162, 0, 155 ),
	RGB565(167, 0, 154 ),
	RGB565(171, 0, 153 ),
	RGB565(175, 1, 152 ),
	RGB565(178, 1, 151 ),
	RGB565(182, 2, 149 ),
	RGB565(185, 4, 149 ),
	RGB565(188, 5, 147 ),
	RGB565(191, 6, 146 ),
	RGB565(193, 8, 144 ),
	RGB565(195, 11, 142 ),
	RGB565(198, 13, 139 ),
	RGB565(201, 17, 135 ),
	RGB565(203, 20, 132 ),
	RGB565(206, 23, 127 ),
	RGB565(208, 26, 121 ),
	RGB565(210, 29, 116 ),
	RGB565(212, 33, 111 ),
	RGB565(214, 37, 103 ),
	RGB565(217, 41, 97 ),
	RGB565(219, 46, 89 ),
	RGB565(221, 49, 78 ),
	RGB565(223, 53, 66 ),
	RGB565(224, 56, 54 ),
	RGB565(226, 60, 42 ),
	RGB565(228, 64, 30 ),
	RGB565(229, 68, 25 ),
	RGB565(231, 72, 20 ),
	RGB565(232, 76, 16 ),
	RGB565(234, 78, 12 ),
	RGB565(235, 82, 10 ),
	RGB565(236, 86, 8 ),
	RGB565(237, 90, 7 ),
	RGB565(238, 93, 5 ),
	RGB565(239, 96, 4 ),
	RGB565(240, 100, 3 ),
	RGB565(241, 103, 3 ),
	RGB565(241, 106, 2 ),
	RGB565(242, 109, 1 ),
	RGB565(243, 113, 1 ),
	RGB565(244, 116, 0 ),
	RGB565(244, 120, 0 ),
	RGB565(245, 125, 0 ),
	RGB565(246, 129, 0 ),
	RGB565(247, 133, 0 ),
	RGB565(248, 136, 0 ),
	RGB565(248, 139, 0 ),
	RGB565(249, 142, 0 ),
	RGB565(249, 145, 0 ),
	RGB565(250, 149, 0 ),
	RGB565(251, 154, 0 ),
	RGB565(252, 159, 0 ),
	RGB565(253, 163, 0 ),
	RGB565(253, 168, 0 ),
	RGB565(253, 172, 0 ),
	RGB565(254, 176, 0 ),
	RGB565(254, 179, 0 ),
	RGB565(254, 184, 0 ),
	RGB565(254, 187, 0 ),
	RGB565(254, 191, 0 ),
	RGB565(254, 195, 0 ),
	RGB565(254, 199, 0 ),
	RGB565(254, 202, 1 ),
	RGB565(254, 205, 2 ),
	RGB565(254, 208, 5 ),
	RGB565(254, 212, 9 ),
	RGB565(254, 216, 12 ),
	RGB565(255, 219, 15 ),
	RGB565(255, 221, 23 ),
	RGB565(255, 224, 32 ),
	RGB565(255, 227, 39 ),
	RGB565(255, 229, 50 ),
	RGB565(255, 232, 63 ),
	RGB565(255, 235, 75 ),
	RGB565(255, 238, 88 ),
	RGB565(255, 239, 102 ),
	RGB565(255, 241, 116 ),
	RGB565(255, 242, 134 ),
	RGB565(255, 244, 149 ),
	RGB565(255, 245, 164 ),
	RGB565(255, 247, 179 ),
	RGB565(255, 248, 192 ),
	RGB565(255, 249, 203 ),
	RGB565(255, 251, 216 ),
	RGB565(255, 253, 228 ),
	RGB565(255, 254, 239 ),
	RGB565(255, 255, 249 )
};
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
	assert(screensize = 320*240*2);
	fbp = (uint16_t *)mmap(0, 
		screensize, 
		PROT_READ | PROT_WRITE, 
		MAP_SHARED, 
		fbfd, 
		0);

	if ((int)fbp == -1) {
		printf("Failed to mmap.\n");
	}

	//memset(screensize, 0, fbp);
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
#else
			assert(h == 120);
			assert(w == 160);
			for(int i = 0; i < 118; i++) {
				for(int j = 0; j < 158; j++) {
					uint16_t val = iron_palette[data[i*160+j]>>1];
					fbp[(i*2)*320+j*2] = val;
					fbp[(i*2)*320+j*2+1] = val;
					fbp[(i*2+1)*320+j*2] = val;
					fbp[(i*2+1)*320+j*2+1] = val;
				}
			}
#endif
			
			frameIdx++;
		}
		
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	
	grabber.stop();

#ifndef WRITE_JPEG
	munmap(fbp, screensize);
	close(fbfd);
#endif

    return EXIT_SUCCESS;
}
