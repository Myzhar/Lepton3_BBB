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

#include "stopwatch.hpp"

#define SAVE_MJPEG 1

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
	
char image_name[32];

#ifdef SAVE_MJPEG 
    cv::VideoWriter writer;
    int fps=9;
    int w=160;
    int h=120;
    
    writer.open( "lepton3.avi", CV_FOURCC('M','J','P','G'), fps, cv::Size(w,h) );
    
    bool writeFrame = writer.isOpened();
#endif
	
    Lepton3 lepton3( "/dev/spidev1.0", 1, deb_lvl );
    
    if( lepton3.enableRadiometry( true ) < 0)
    {
        cout << "Failed to enable radiometry" << endl;
    }
    else
    {
        cout << " * Radiometry enabled " << endl;
    }
    
	lepton3.start();
	
	uint64_t frameIdx=0;	
	
	StopWatch stpWtc;
	
	stpWtc.tic();
	
	while(!close)
    {	
        uint16_t min;
        uint16_t max;	
        unsigned short* data = lepton3.getLastFrame( &min, &max );
		
        if( data )
		{
		    double period_usec = stpWtc.toc();
		    stpWtc.tic();
		    
		    double freq = (1000.*1000.)/period_usec; 
		    cv::Mat frame16( 120, 160, CV_16UC1 );
		    
		    memcpy( frame16.data, data, 160*120*sizeof(unsigned short) );
            
            // >>>>> Rescaling/Normalization to 8bit
            double diff = static_cast<double>(max - min);
		    double scale = 255./diff;
		    
		    frame16 -= min;
		    frame16 *= scale; 

            cv::Mat frame8;
            frame16.convertTo( frame8, CV_8UC1 ); 
            // <<<<< Rescaling/Normalization to 8bit
			
#ifdef SAVE_MJPEG
            if(writeFrame)
            {
                cv::cvtColor(frame8,frame8, CV_GRAY2BGR );
                writer.write(frame8);
            }
#else
			sprintf(image_name, "IMG_%.6lu.png", frameIdx);
			string imgStr = image_name;
            cv::imwrite( imgStr, frame8 );       
               
            if( deb_lvl>=Lepton3::DBG_INFO  )
			{
                cout << "> " << imgStr << endl;
            }
#endif
			
			frameIdx++;
			
			if( deb_lvl>=Lepton3::DBG_INFO  )
			{
			    cout << "> Frame period: " << period_usec <<  " usec - FPS: " << freq << endl;
			}
        }
		
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	
	lepton3.stop();
	
#ifdef SAVE_MJPEG
    writer.release();
#endif

    return EXIT_SUCCESS;
}
