#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include "lepton3_grabber.hpp"

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
	
	l3_grabber grabber;
	
	grabber.start();
	
	while(!close)
	{
		int w;
		int h;
		
		const char* data = grabber.getLastFrame( &w, &h );
		
		if( data && w!=-1 && h!=-1 )
		{
			
			cv::Mat frame( h,w, CV_8UC1 );
			memcpy( frame.data, data, sizeof(data) );
		}
	}

    return EXIT_SUCCESS;
}
