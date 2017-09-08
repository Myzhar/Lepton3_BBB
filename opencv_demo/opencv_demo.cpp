#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <string>
#include <chrono>
#include <thread>
#include "Lepton3.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
	
    Lepton3 lepton3( "/dev/spidev1.0", 1, Lepton3::DBG_FULL );
	lepton3.start();
	
	uint64_t frameIdx=0;
	char image_name[32];
	
	while(!close)
    {
		
        unsigned short* data = lepton3.getLastFrame( );
		
        if( data )
		{
            sprintf(image_name, "IMG_%.6lu.jpg", frameIdx);
			string imgStr = image_name;
			
            cv::Mat frame16( 120, 160, CV_16UC1 );

            memcpy( frame16.data, data, 160*120 );

            cv::Mat frame8;
            frame16.convertTo( frame8, CV_8UC1, 1./256. );

            cv::normalize( frame8, frame8, 1, 0, cv::NORM_MINMAX );
			
            cv::imwrite( imgStr, frame8 );
			
			frameIdx++;
        }
		
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	
	lepton3.stop();

    return EXIT_SUCCESS;
}
