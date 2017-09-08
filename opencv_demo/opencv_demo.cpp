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
	
	Lepton3::DebugLvl deb_lvl = Lepton3::DBG_NONE;
	
	if( argc == 2 )
	{
	    int dbg = atoi(argv[1]);
	    
	    switch( dbg )
	    {
	    case 1:
	        deb_lvl = Lepton3::DBG_INFO;
	        break;
	        
	    case 2:
	        deb_lvl = Lepton3::DBG_FULL;
	        break;
	        
	    default:
	    case 0:
	        deb_lvl = Lepton3::DBG_NONE;
	        break;	    
	    }
	}
	
    Lepton3 lepton3( "/dev/spidev1.0", 1, deb_lvl );
	lepton3.start();
	
	uint64_t frameIdx=0;
	char image_name[32];
	
	while(!close)
    {	
        uint16_t min;
        uint16_t max;	
        unsigned short* data = lepton3.getLastFrame( &min, &max );
		
        if( data )
		{
            sprintf(image_name, "IMG_%.6lu.png", frameIdx);
			string imgStr = image_name;
			
            cv::Mat frame16( 120, 160, CV_16UC1, data );
            
            // >>>>> Rescaling/Normalization to 8bit
            double diff = static_cast<double>(max - min);
		    double scale = 255./diff;
		    
		    frame16 -= min;
		    frame16 *= scale; 

            cv::Mat frame8;
            frame16.convertTo( frame8, CV_8UC1 ); 
            // <<<<< Rescaling/Normalization to 8bit
			
            cv::imwrite( imgStr, frame8 );
			
			frameIdx++;
			
			if( deb_lvl>=Lepton3::DBG_INFO  )
			{
			    cout << "> " << imgStr << endl;
			}
        }
		
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	
	lepton3.stop();

    return EXIT_SUCCESS;
}
