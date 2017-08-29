#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <string>
#include <chrono>
#include <thread>
#include "Lepton3.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

static bool close = false;

void close_handler(int s)
{
	if(s==2)
	{
		cout << endl << "Ctrl+C pressed..." << endl;		
		close = true;
	} 
}

int main (int argc, char *argv[])
{
	cout << "OpenCV demo for Lepton3 on BeagleBoard Blue" << endl;
	
	// >>>>> Enable Ctrl+C
	struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = close_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    // <<<<< Enable Ctrl+C
	
	Lepton3 lepton3;
	
	lepton3.start();
	
	uint64_t frameIdx=0;
	char image_name[32];
	
	while(!close)
	{
		int w;
		int h;
		
		/*const char* data = grabber.getLastFrame( &w, &h );
		
		if( data && w!=-1 && h!=-1 )
		{
			sprintf(image_name, "IMG_%.6d.jpg", frameIdx);
			string imgStr = image_name;
			
			cv::Mat frame( h,w, CV_8UC1 );
			memcpy( frame.data, data, w*h );
			
			cv::imwrite( image_name, frame );
			
			cout << "Saved: " << image_name << "[" << w << "x" << h << "]" << endl;
			
			frameIdx++;
		}*/
		
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	
	lepton3.stop();

    return EXIT_SUCCESS;
}
